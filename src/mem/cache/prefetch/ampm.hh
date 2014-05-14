// AMPM implementation by Ishii et al. from DPC 2009
#ifndef __MEM_CACHE_PREFETCH_AMPM_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_AMPM_PREFETCHER_HH__

#include <climits>

#include "mem/cache/prefetch/base.hh"
#include "params/AMPMPrefetcher.hh"

#define ASSERT(X)

static const int BLK_WIDTH =  6;
static const int IDX_WIDTH =  8;
static const int TAG_WIDTH = 18;

static const int BLK_BITS = BLK_WIDTH;
static const int IDX_BITS = IDX_WIDTH + BLK_BITS;
static const int TAG_BITS = TAG_WIDTH + IDX_BITS;

static const uint64_t BLK_SIZE = ((uint64_t)1) << BLK_BITS;
static const uint64_t IDX_SIZE = ((uint64_t)1) << IDX_BITS;
static const uint64_t TAG_SIZE = ((uint64_t)1) << TAG_BITS;

static const uint64_t BLK_MASK = (((uint64_t)1) << BLK_WIDTH)-1;
static const uint64_t IDX_MASK = (((uint64_t)1) << IDX_WIDTH)-1;
static const uint64_t TAG_MASK = (((uint64_t)1) << TAG_WIDTH)-1;

static const int PREF_MSHR_SIZE = 32;

// Generate Index and Tag
#define GenTag(addr) (((addr) >> IDX_BITS) & TAG_MASK)
#define GenIdx(addr) (((addr) >> BLK_BITS) & IDX_MASK)

////////////////////////////////////////////////////
// Definition of Constants.
////////////////////////////////////////////////////

// Access Map Size
static const int MAP_Size  = 1 << IDX_WIDTH;

// Memory Access Map Table Size
// #define REALISTIC
// Our submission is realistic configuration

#ifdef REALISTIC
// Realistic Configuration
// 13 way set associative table
static const int NUM_SET =  4;
static const int NUM_WAY = 13;

#else
// Idealistic Configuration
// Full associative table
static const int NUM_SET =  1;
//static const int NUM_WAY = 52;
static const int NUM_WAY = 256;

#endif

// TABLE SIZE is 52 entries because of the storage limitation.
// If  we have more budget, we will emply 64 entries.
// And we have less budget, we will emply 32 entries.
static const int TABLE_SIZE = NUM_SET * NUM_WAY;

// Threshold
// They are only the constansts.
static const int AP_THRESHOLD = 8;
static const int SE_THRESHOLD = 128;
static const int CA_THRESHOLD = 256;


//
// Prefetch MSHR
// This mshr is provided for handling status of prefetch requests.
// Entries of prefetch MSHR are counted for 32K bit budget count.
//
struct PrefetchMissStatusHandlingRegister;
struct MemoryAccessMapTable;

class AMPMPrefetcher : public BasePrefetcher
{
    // MSHR for prefetch requests(901 bits)
    PrefetchMissStatusHandlingRegister *mshrpf;
    // Memory Access Map Table(29147 bits)
    MemoryAccessMapTable *pref;

    // degree
    int degree;


    // Prefetch requests for generating prefetches.
    // * These registers are overwritten
    // * only when the new demand requests are reached to the prefetcher.
    int NumFwdPref[2];              // 5 bit counter
    int NumBwdPref[2];             // 5 bit counter
    bool FwdPrefMap[2][MAP_Size/2]; // 128 bit register
    bool BwdPrefMap[2][MAP_Size/2]; // 128 bit register
    uint64_t AddressRegister[2]; // 26 bit register
    // Pipeline registers total budget : 292 bit

    // Total Budget Size: 32036 bit
    void init_ampm_prefetcher();

    public:

    AMPMPrefetcher(const Params *p)
        : BasePrefetcher(p), degree(p->degree)
    {
        init_ampm_prefetcher();
    }
    void IssuePrefetchCandidates(uint64_t key, uint64_t addr, bool mshr_hit, bool hit );
};    


struct PrefetchMissStatusHandlingRegisterEntry{
  // Budget Count

  // Issued bit :  1 bit
  // Valid bit  :  1 bit
  // Address bit: 26 bit
  // Total Count: 28 bit
  bool Valid;
  bool Issue;
  uint64_t Addr;

public:
  // Budget() returns an amount of storage for
  // implementing prefetch algorithm.
  int Budget(){ return 0; }

  // Initialize MSHR entry
  void Init(){
    Valid = false;
    Issue = false;
    Addr  = (uint64_t)0;
  }

  // Returns hit or miss with MSHR entry.
  bool Hit(uint64_t addr) {
    // NULL pointer handling
    if(addr==0) return true;

    // Check Address
    addr ^= Addr;
    addr &= ~BLK_MASK;
    return Valid && (addr == (uint64_t)0);
  }

  // Assign Entry
  void Entry(uint64_t addr){
    Valid = true;
    Issue = false;
    Addr  = addr;
  }

  // Assign Entry
  void IssuePrefetch( AMPMPrefetcher* prefetcher);
};

struct PrefetchMissStatusHandlingRegister {
  AMPMPrefetcher* prefetcher;
  // Total Budget Size : 901 bits

  // MSHR Entry Size is 32
  int ptr; // 5 bits
  PrefetchMissStatusHandlingRegisterEntry mshr[PREF_MSHR_SIZE]; // 28 bits x 32

  PrefetchMissStatusHandlingRegister(AMPMPrefetcher* prefetcher_) 
        : prefetcher(prefetcher_)
  {
    ptr = 0;
    for(int i=0; i<PREF_MSHR_SIZE; i++){
      mshr[i].Init();
    }
  }

  bool Full(){
    return mshr[ptr].Valid;
  }

  bool Search(uint64_t addr){
    // Search Previous Memory Access
    for(int i=0; i<PREF_MSHR_SIZE; i++){
      // When the appropriate access is found, the access is merged.
      if(mshr[i].Hit(addr)) return true;
    }
    return false;
  }

  void Issue( uint64_t addr){
    // Check In-flight memory access
    if(Search(addr)) return;

    // When the appropriate access cannot be found,
    // the access will issue for main memory.
    mshr[ptr].Entry(addr);

    ptr = (ptr + 1) % PREF_MSHR_SIZE;
  }

  void PrefetchHousekeeping(){
      for(int i=0; i<PREF_MSHR_SIZE; i++){
          int p = (ptr + i + 1) % PREF_MSHR_SIZE;
          if((mshr[p].Valid) && (!mshr[p].Issue)) {
              // When the entry does not issued,
              // prefetch MSHR issues request to Memory Unit.
              mshr[p].IssuePrefetch(prefetcher);
              return;
          }
      }
  }
};

////////////////////////////////////////////////////
// Definition of Constants.
////////////////////////////////////////////////////

// Memory access map status is implemented as 2bit state machine
enum MemoryAccessMapState {
  INIT    , // INIT     Status (00)
  ACCESS  , // ACCESS   Status (01)
  PREFETCH, // PREFETCH Status (10)
  SUCCESS , // SUCCESS  Status (11)
};


struct MemoryAccessMap {
  int LRU; // 6 bits LRU information

  uint64_t LastAccess; // Timer(16 bits)
  uint64_t NumAccess;  // Access counter(4 bits)
  uint64_t Tag;    // 18 bit address tag(18 bits)
  uint64_t access_freq;
  uint64_t miss_freq;

  // 2 bit state machine x 256 = 512 bit Map
  enum MemoryAccessMapState AccessMap[MAP_Size];

  // Total Budget Count : 556 bits

  /////////////////////////////////////////////////////
  // Constructer & Copy Constructer
  /////////////////////////////////////////////////////
  MemoryAccessMap(uint64_t t=0) : Tag(t) { }
  MemoryAccessMap(const MemoryAccessMap& ent) : Tag(ent.Tag) { }
  ~MemoryAccessMap(){}

  /////////////////////////////////////////////////////
  // Read prefetch infomation
  /////////////////////////////////////////////////////
  bool Hit(uint64_t addr) {
    return Tag == GenTag(addr);
  }

  bool AlreadyAccess(uint64_t addr) {
    return (Hit(addr) && (AccessMap[GenIdx(addr)] != INIT));
  }

  bool PrefetchHit(uint64_t addr) {
    return (Hit(addr) && (AccessMap[GenIdx(addr)] == INIT));
  }

  void Read(enum MemoryAccessMapState *ReadMap) {
    for(int i=0; i<MAP_Size; i++){
      ReadMap[i] = AccessMap[i];
    }
  }
  
  /////////////////////////////////////////////////////
  // Updating methods...
  /////////////////////////////////////////////////////

  // Assign new Czone to this map.
  void Entry(uint64_t addr){
    // Assign & Initializing Entry
    Tag = GenTag(addr);
    for(int i=0; i<MAP_Size; i++) { AccessMap[i] = INIT; }
    LastAccess = 0;
    NumAccess = 0;
    access_freq = 0;
    miss_freq = 0;
  }

  // Issuing prefetch request
  bool IssuePrefetchSub(uint64_t addr){
    if(!Hit(addr)) return false;

    // L2 Access Control
    ASSERT(AccessMap[GenIdx(addr)] == INIT);
    AccessMap[GenIdx(addr)]=PREFETCH;
    return true;
  }

  // Updating state machine
  void Update(uint64_t addr){
    ASSERT(Hit(addr));

    switch(AccessMap[GenIdx(addr)]) {
    case INIT    : AccessMap[GenIdx(addr)] = ACCESS; break;
    case ACCESS  : return;
    case PREFETCH: AccessMap[GenIdx(addr)] = SUCCESS; break;
    case SUCCESS : return;
    default    : ASSERT(false); break;
    }

    if(NumAccess >= 15) {
      NumAccess  = 8;
      LastAccess = LastAccess >> 1;
    } else {
      NumAccess++;
    }
    
    return;
  }

  /////////////////////////////////////////////////////
  // Profiling methods
  /////////////////////////////////////////////////////
  int NumInit() {
    int Succ = 0;
    for(int i=0; i<MAP_Size; i++) {
      Succ += (AccessMap[i] == INIT ? 1 : 0);
    }
    return Succ;
  }
  int NumSuccPref() {
    int Succ = 0;
    for(int i=0; i<MAP_Size; i++) {
      Succ += (AccessMap[i] == SUCCESS ? 1 : 0);
    }
    return Succ;
  }
  int NumAccesses() {
    int Succ = 0;
    for(int i=0; i<MAP_Size; i++) {
      Succ += (AccessMap[i] == ACCESS ? 1 : 0);
    }
    return Succ;
  }
  int NumFailPref() {
    int Succ = 0;
    for(int i=0; i<MAP_Size; i++) {
      Succ += (AccessMap[i] == PREFETCH ? 1 : 0);
    }
    return Succ;
  }

  // Returns stream length of the prefetch 
  // * The stream length is decided from access history
  int MaxAccess() {
    int req = (NumAccess * 256) / (1 + LastAccess);
    int max = 7;
    return max > req ? req : max;
  }

  /////////////////////////////////////////////////////
  // Housekeeping
  /////////////////////////////////////////////////////

  // Updates some counters.
  void Housekeeping(){
    LastAccess++;
    if(LastAccess > 65536) {
      LastAccess = 65536;
    }
  }
};

struct MemoryAccessMapTable {
  AMPMPrefetcher* prefetcher;
  // These flags uses 3 bits
  bool Aggressive;
  bool SaveEntry;
  bool ConflictAvoid;

  // These counter uses 128 bits
  int  NeedEntry;
  int  Conflict;
  int  PrefFail;
  int  PrefSucc;

  // 54 entries table (556 x 52)
  MemoryAccessMap Table[NUM_SET][NUM_WAY];

  // Total Budget Count = 29147 bits

  /////////////////////////////////////////////
  // Constructor & Copy Constructor
  /////////////////////////////////////////////

  MemoryAccessMapTable(AMPMPrefetcher* prefetcher_)
        : prefetcher(prefetcher_)
  {
    NeedEntry     = 64;
    Conflict      = 16;
    SaveEntry     = false;
    ConflictAvoid = false;
    Aggressive    = true;

    for(int Idx=0; Idx<NUM_SET; Idx++) {
      for(int Way=0; Way<NUM_WAY; Way++) {
	Table[Idx][Way].LRU = Way;
	Table[Idx][Way].Entry(0);
      }
    }
  }

  ~MemoryAccessMapTable() { }

  /////////////////////////////////////////////
  // Basic Functions.
  /////////////////////////////////////////////

  // Status Handling Function
  bool IsAccessed(MemoryAccessMapState state) {
    switch(state) {
    case INIT    : return false;
    case ACCESS  : return true;
    case PREFETCH: return Aggressive;
    case SUCCESS : return true;
    default    : ASSERT(false); return false;
    }
  }
  
  bool IsCandidate(MemoryAccessMapState state, int index, int passivemode) {
    return ((state == INIT) &&
	    ((index >= MAP_Size && index < (2*MAP_Size)) || !passivemode));
  }

  // Read Memory Access Map from Table
  MemoryAccessMap* AccessEntry(uint64_t addr);

  // Update Memory Access Map when the entry is on the table.
  void UpdateEntry(uint64_t addr); 

  // Read Prefetch Candidates from Memory Access Map
  // This function does not has any memories.
  void IssuePrefetch(uint64_t addr, bool MSHR_hit,
		     bool *FwdPrefMap, bool *BwdPrefMap,
		     int  *NumFwdPref, int *NumBwdPref, bool hit=true) ;

  void Housekeeping();
};

#endif // __MEM_CACHE_PREFETCH_AMPM_PREFETCHER_HH__


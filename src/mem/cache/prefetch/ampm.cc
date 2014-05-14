// AMPM Prefetcher

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/ampm.hh"

void AMPMPrefetcher::init_ampm_prefetcher()
{
    mshrpf = new PrefetchMissStatusHandlingRegister(this);
    pref = new MemoryAccessMapTable(this);
    NumFwdPref[0] = NumFwdPref[1] = NumBwdPref[0] = NumBwdPref[1] = 0;
}


    // Assign Entry
void PrefetchMissStatusHandlingRegisterEntry::IssuePrefetch(AMPMPrefetcher* prefetcher){
    ASSERT(Valid && !Issue);
    Issue = true;

    prefetcher->InitPrefetch(Addr);

    Valid = false;
    Issue = false;
}

/////////////////////////////////////////////
// Read & Update memory access map
/////////////////////////////////////////////

// Read Memory Access Map from Table
MemoryAccessMap* MemoryAccessMapTable::AccessEntry(uint64_t addr) {
    int Oldest = 0;

    //stats.dcache.ampm.read++;
    int Idx = GenTag(addr) % NUM_SET;
    ASSERT(Idx >= 0 && Idx < NUM_SET);
    for(int Way=0; Way<NUM_WAY; Way++) {
        if(Table[Idx][Way].Hit(addr)) {
            // Find Entry
            for(int i=0; i<NUM_WAY; i++) {
                if(Table[Idx][i].LRU < Table[Idx][Way].LRU) {
                    Table[Idx][i].LRU++;
                    ASSERT(Table[Idx][i].LRU < NUM_WAY);
                }
            }
            Table[Idx][Way].LRU = 0;
            return (&(Table[Idx][Way]));
        }

        if(Table[Idx][Way].LRU > Table[Idx][Oldest].LRU) {
            Oldest = Way;
        }
    }

    ASSERT(Table[Idx][Oldest].LRU == (NUM_WAY-1));

    // Find Entry
    for(int i=0; i<NUM_WAY; i++) {
        if(Table[Idx][i].LRU < Table[Idx][Oldest].LRU) {
            Table[Idx][i].LRU++;
            ASSERT(Table[Idx][i].LRU < NUM_WAY);
        }
    }
    PrefFail += Table[Idx][Oldest].NumFailPref();
    PrefSucc += Table[Idx][Oldest].NumSuccPref();
    Table[Idx][Oldest].Entry(addr); 
    //stats.dcache.ampm.evict++;

    Table[Idx][Oldest].LRU = 0;
    return (&(Table[Idx][Oldest]));
}

// Update Memory Access Map when the entry is on the table.
void MemoryAccessMapTable::UpdateEntry(uint64_t addr) {
    // Issueing Prefetch
    //stats.dcache.ampm.write++;
    int Idx = GenTag(addr) % NUM_SET;
    ASSERT(Idx >= 0 && Idx < NUM_SET);
    for(int Way=0; Way<NUM_WAY; Way++) {
        if(Table[Idx][Way].Hit(addr)) {
            Table[Idx][Way].IssuePrefetchSub(addr);
            return;
        }
    }
}

///////////////////////////////////////////////////
// Read access map & generate prefetch candidates.
///////////////////////////////////////////////////


// Read Prefetch Candidates from Memory Access Map
// This function does not has any memories.
void MemoryAccessMapTable::IssuePrefetch(uint64_t addr, bool MSHR_hit,
            bool *FwdPrefMap, bool *BwdPrefMap,
            int  *NumFwdPref, int *NumBwdPref, bool hit) {
    //////////////////////////////////////////////////////////////////
    // Definition of Temporary Values
    //////////////////////////////////////////////////////////////////

    // These variables are not counted as a storage
    // Since these are used as a wire (not register)
    MemoryAccessMap *ent_l, *ent_m, *ent_h;
    static enum MemoryAccessMapState ReadMap[MAP_Size * 3];
    static enum MemoryAccessMapState FwdHistMap[MAP_Size];
    static enum MemoryAccessMapState BwdHistMap[MAP_Size];
    static bool FwdCandMap[MAP_Size];
    static bool BwdCandMap[MAP_Size];
    static bool FwdPrefMapTmp[MAP_Size/2];
    static bool BwdPrefMapTmp[MAP_Size/2];
    static int  NumFwdPrefTmp;
    static int  NumBwdPrefTmp;

    bool PassiveMode = SaveEntry || ConflictAvoid; // Mode

    //////////////////////////////////////////////////////////////////
    // Read & LRU update for access table
    //////////////////////////////////////////////////////////////////
    if(PassiveMode) {
        ent_l = NULL;
        ent_h = NULL;
    } else {
        ent_l = AccessEntry( addr - IDX_SIZE);
        ent_h = AccessEntry( addr + IDX_SIZE);
    }
    ent_m = AccessEntry(addr);

    //////////////////////////////////////////////////////////////////    
    // Following Cases are not prefered.
    //
    // 1. AccessMap==0 && PrefetchBit==1 (Prefetched)
    //      -> There are not enough Entry Size
    // 2. L2 Miss && Already Accessed && MSHR Miss (Finished filling)
    //      -> L2 Cache conflict miss is detected.
    //
    // Our prefetcher detect these situation and profiling these case.
    // When the frequenty exceed some threshold,
    // the profiler changes the prefetch mode.
    //////////////////////////////////////////////////////////////////
    /*   bool PF_succ = prefetcher.GetPrefetchBit(2, addr) == 1; // Prefetch successed
         bool L2_miss = prefetcher.GetPrefetchBit(2, addr)  < 0; // L2 miss detection
         prefetcher.UnSetPrefetchBit(2, addr); // Clear Prefetch Bit

         bool NE_INC = PF_succ && ent_m->PrefetchHit(addr);
         bool CF_INC = L2_miss && ent_m->AlreadyAccess(addr) && !MSHR_hit;
         NeedEntry += NE_INC ? 1 : 0;
         Conflict  += CF_INC ? 1 : 0;
     */

    //////////////////////////////////////////////////////////////////
    // Prefetching Part...
    //////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////
    // Read Entry (Read & Concatenate Accessed Maps)
    //////////////////////////////////////////////////////////////////
    for(int i=0; i<MAP_Size; i++) { ReadMap[i] = INIT; }

    if(ent_l) ent_l->Read(&ReadMap[0 * MAP_Size]);
    if(ent_m) ent_m->Read(&ReadMap[1 * MAP_Size]);
    if(ent_h) ent_h->Read(&ReadMap[2 * MAP_Size]);

    //////////////////////////////////////////////////////////////////
    // Generating Forwarding Prefetch
    //////////////////////////////////////////////////////////////////
    if(ConflictAvoid) {
        NumFwdPrefTmp = 2;
    } else {
        NumFwdPrefTmp = 2
            + (ent_m ? ent_m->MaxAccess() : 0);
        + (PassiveMode && ent_l ? 0 : ent_l->MaxAccess());
    }

    // This block should be implemented in simple shifter
    for(int i=0; i<MAP_Size; i++) {
        int index_p = MAP_Size + GenIdx(addr) + i;
        int index_n = MAP_Size + GenIdx(addr) - i;
        FwdHistMap[i] = ReadMap[index_n];
        FwdCandMap[i] = IsCandidate(ReadMap[index_p], index_p, PassiveMode);
    }

    // This for statement can be done in parallel
    for(int i=0; i<MAP_Size/2; i++) {
        FwdPrefMapTmp[i] =
            FwdCandMap[i] &&
            IsAccessed(FwdHistMap[i]) &&
            ( IsAccessed(FwdHistMap[2*i+0]) ||
              IsAccessed(FwdHistMap[2*i+1]) );
    }

    //////////////////////////////////////////////////////////////////
    // Generating Backward Prefetch
    //////////////////////////////////////////////////////////////////

    if(ConflictAvoid) {
        NumBwdPrefTmp = 2;
    } else {
        NumBwdPrefTmp = 2
            + (ent_m ? ent_m->MaxAccess() : 0);
        + (PassiveMode && ent_h ? 0 : ent_h->MaxAccess());
    }

    for(int i=0; i<MAP_Size; i++) {
        int index_p = MAP_Size + GenIdx(addr) + i;
        int index_n = MAP_Size + GenIdx(addr) - i;
        BwdHistMap[i] = ReadMap[index_p];
        BwdCandMap[i] = IsCandidate(ReadMap[index_n], index_n, PassiveMode);
    }

    // This for statement can be done in parallel
    for(int i=0; i<MAP_Size/2; i++) {
        BwdPrefMapTmp[i] =
            BwdCandMap[i] &&
            IsAccessed(BwdHistMap[i]) &&
            ( IsAccessed(BwdHistMap[2*i+0]) ||
              IsAccessed(BwdHistMap[2*i+1]) );
    }

    //////////////////////////////////////////////////////////////////
    // Update Entry
    //////////////////////////////////////////////////////////////////

    // stats.dcache.ampm.write++;
    ent_m->Update(addr);
    ent_m->access_freq++;
    if(!hit)
        ent_m->miss_freq++;

    //////////////////////////////////////////////////////////////////
    // Setting Output Values (Copy to Arguments)
    //////////////////////////////////////////////////////////////////

    *NumFwdPref = NumFwdPrefTmp;
    *NumBwdPref = NumBwdPrefTmp;
    for(int i=0; i<MAP_Size/2; i++) {
        BwdPrefMap[i] = BwdPrefMapTmp[i];
        FwdPrefMap[i] = FwdPrefMapTmp[i];
    }
}


///////////////////////////////////////////////////
// Housekeeping Function
///////////////////////////////////////////////////

void MemoryAccessMapTable::Housekeeping(){
    // Housekeeping for each entry.
    for(int Idx=0; Idx<NUM_SET; Idx++) {
        for(int Way=0; Way<NUM_WAY; Way++) {
            Table[Idx][Way].Housekeeping();
        }
    }
/*
    uint64_t cycle = 1;
    // Aggressive Prefetching Mode
    // This mode is enabled when the 
    // (prefetch success count > 8 * prefetch fail count) is detected.
    // If this mode is enabled,
    // the prefetcher assume PREFETCH state as the ACCESS state.
    if((cycle & (((uint64_t)1 << 18)-1)) == 0) {
        if(PrefSucc > AP_THRESHOLD * PrefFail) {
            // printf("Aggressive Prefetch Mode\n");
            Aggressive = true;
        } else if(PrefSucc < (AP_THRESHOLD/2) * PrefFail) {
            // printf("Normal Prefetch Mode\n");
            Aggressive = false;
        }
        PrefSucc >>= 1;
        PrefFail >>= 1;
    }

    // Profiling Execution Status
    // This mode is enabled when the replacement of an access table entry
    // occurs frequently. When this mode is enabled,
    // the prefetcher stops reading neighbor entries.
    // This feature reduces the unprefered access entry replacements.
    if((cycle & (((uint64_t)1 << 18)-1)) == 0) {
        if(NeedEntry > SE_THRESHOLD) {
            // printf("Save Entry Mode\n");
            SaveEntry = true;
        } else if(NeedEntry > (SE_THRESHOLD/4)) {
            // printf("Normal Entry Mode\n");
            SaveEntry = false;
        }
        NeedEntry >>= 1;
    }

    // Profiling Execution Status
    // This mode is enabled when the L2 conflict misses are detected frequentry.
    // When the conflict avoidance mode is enabled,
    // the prefetcher reduces the number of prefetch requests.
    if((cycle & (((uint64_t)1 << 18)-1)) == 0) {
        if(Conflict > CA_THRESHOLD) {
            // printf("Conflict Avoidance Mode\n");
            ConflictAvoid = true;
        } else if(Conflict > (CA_THRESHOLD/4)) {
            // printf("Normal Conflict Mode\n");
            ConflictAvoid = false;
        }
        Conflict >>= 1;
    }
*/
}

void AMPMPrefetcher::IssuePrefetchCandidates(uint64_t key, uint64_t trigger_addr, bool mshr_hit, bool hit )
{
  //  ptl_logfile << " Trigger " << (void*)trigger_addr << endl, flush;

    /////////////////////////////////////////////////////
    // Stage 1. Access to Memory Access Map Table
    /////////////////////////////////////////////////////
    // i is the number of trigger requests AMPM can take in a cycle 
    for(int i=0; i <1; i++) 
    {
        // Memory Access Map Table Access
        // Read prefetch candidate from Memory Access Map Table
        pref->IssuePrefetch(trigger_addr, false,
                FwdPrefMap[i], BwdPrefMap[i],
                &NumFwdPref[i], &NumBwdPref[i], hit);
        AddressRegister[i] = trigger_addr;
        AddressRegister[i] &= ~(BLK_SIZE-1);
    }


    /////////////////////////////////////////////////////
    // Stage 2. Issue Prefetch Request to MSHR
    /////////////////////////////////////////////////////

    // This loop is implemented as a priority encoder
    for(int k=0; k<1; k++){
        for(unsigned int i=1, count=0;
                ( NumFwdPref[k] && (count < degree) &&
                  (!mshrpf->Full()) && (i<(MAP_Size/2)) );i++) {
            if(FwdPrefMap[k][i]) {
                uint64_t PrefAddress = AddressRegister[k] + (i * BLK_SIZE);
                FwdPrefMap[k][i] = false;

                // Update Memory Access Map
                pref->UpdateEntry(PrefAddress);

                mshrpf->Issue  (PrefAddress);
                NumFwdPref[k]--;
                count++;
            }
        }

        // This loop is implemented as a priority encoder
        for(unsigned int i=1, count=0;
                ( NumBwdPref[k] && (count < degree) &&
                  (!mshrpf->Full()) && (i<(MAP_Size/2)) );
                i++) {
            if(BwdPrefMap[k][i] && (AddressRegister[k] > (i * BLK_SIZE))) {
                uint64_t PrefAddress = AddressRegister[k] - (i * BLK_SIZE);
                BwdPrefMap[k][i] = false;

                // Update Memory Access Map
                pref->UpdateEntry(PrefAddress);

                mshrpf->Issue  (PrefAddress);
                NumBwdPref[k]--;
                count++;
            }
        }
    }

    /////////////////////////////////////////////////////
    // Stage 3. Issue Prefetch from MSHR
    /////////////////////////////////////////////////////
    mshrpf->PrefetchHousekeeping();

    /////////////////////////////////////////////////////
    // Housekeeping
    /////////////////////////////////////////////////////
    pref->Housekeeping();
}

AMPMPrefetcher*
AMPMPrefetcherParams::create()
{
   return new AMPMPrefetcher(this);
}


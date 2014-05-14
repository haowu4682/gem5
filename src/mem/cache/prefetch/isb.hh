// ISB implementation by Akanksha Jain
#ifndef __ISB_H__
#define __ISB_H__

#include "mem/cache/prefetch/base.hh"
#include "params/ISBPrefetcher.hh"
#include "mem/cache/prefetch/training_unit.hh"
#include "mem/cache/prefetch/isb-off-chip.hh"

struct StreamBufferEntry
{
    uint64_t start_str_addr;
    uint16_t size;
    bool access_map[STREAM_MAX_LENGTH];
    public:

    void reset(){
        size = 0;
        for(int i=0; i<STREAM_MAX_LENGTH; i++)
            access_map[i]=0;
    }
    StreamBufferEntry(){
        reset();
    }
    void init(uint64_t addr){
        start_str_addr = addr;
        reset();
    }
    void update_state(uint64_t offset){
        access_map[offset] = 1;
    }
    void print(){
        ptl_logfile << (void*)start_str_addr << " -> ";
        for(int i=0; i<STREAM_MAX_LENGTH; i++)
            ptl_logfile << access_map[i] << " ";
        ptl_logfile << endl;
    }
};

struct StreamBuffer : public FullyAssociativeArray<uint64_t, StreamBufferEntry, 16>
{
    typedef FullyAssociativeArray<uint64_t, StreamBufferEntry, 16> base_t;
    StreamBuffer() : base_t() {}

    void reset() {
        base_t::reset();
    }

    uint64_t get_tag(uint64_t addr){
        return (addr>>8);
    }

    void update_state(uint64_t addr){
        StreamBufferEntry* entry = base_t::probe(get_tag(addr));
        if(!entry){
            entry = base_t::select(get_tag(addr));
            entry->init(get_tag(addr));            
        }

        entry->update_state(addr%256);
    }
    void print(uint64_t addr){
        ptl_logfile << (void*)addr << endl;
        StreamBufferEntry* entry = base_t::probe(get_tag(addr));
        assert(entry);
        entry->print();
    }
};

#define BUFFER_SIZE 128
struct PrefetchBuffer
{
    uint32_t buffer[BUFFER_SIZE];
    bool valid[BUFFER_SIZE];
    uint32_t next_index;

    void reset(){
        for(uint8_t i=0; i<BUFFER_SIZE; i++)
            valid[i] = false;
        next_index = 0;
    }
    void add(uint32_t addr){
        buffer[next_index] = addr;
        valid[next_index] = true;
        next_index = (next_index + 1)%BUFFER_SIZE;
    }

    void issue(uint8_t i){
        assert(valid[i]);
        valid[i] = false;
    }
};

struct ISBPrefetcher : public BasePrefetcher
{
    uint32_t degree;
    uint32_t lookahead;

    TUCache training_unit; 
    AddressMappingCache amc;
    uint64_t alloc_counter;
    StreamBuffer sb;
    AddressEncoder addr_encoder;
    CorrMatrixType off_chip_corr_matrix;
    uint64_t last_page;

    uint64_t stream_divergence_count;
    uint64_t candidate_tlb_miss;
    uint64_t candidate_diff;
    PrefetchBuffer prefetch_buffer;

    //ISBStats new_stats;

    void inform_tlb_eviction(uint64_t, uint32_t);
    uint32_t isb_train(uint32_t str_addr_A, uint16_t encoded_phy_addr_B);
    void isb_predict(uint64_t, uint32_t);
    bool access_training_unit(uint64_t key, uint64_t& addr_A, uint32_t& str_addr_A, uint64_t addr_B);
    void update_training_unit(uint64_t key, uint64_t phy_addr, uint32_t str_addr);

    public :
    //ISBPrefetcher(char* n1, CacheController* controller_handle_, int cacheLineBits_/*, Statable* parent_stats*/) : 
    //    PrefetcherInterface(n1, controller_handle_, cacheLineBits_)
    ISBPrefetcher(const Params *p)
        : BasePrefetcher(p), degree(p->degree), lookahead(p->lookahead),
        amc()
//    , new_stats("isb", parent_stats)
    {
        training_unit.reset();
        amc.reset();
        alloc_counter = 0;
        addr_encoder.reset();
        sb.reset();
        last_page = 0;

        stream_divergence_count = 0;
        candidate_tlb_miss = 0;
        candidate_diff = 0;
        prefetch_buffer.reset(); 
    }

    void IssuePrefetchCandidates( uint64_t key, uint64_t addr, bool mshr_hit, bool hit );
    bool InitMetaDataWrite(Addr addr, int delay = 0);
    bool InitMetaDataRead(Addr addr, int delay = 0);

    uint32_t assign_structural_addr(); 
};

#endif


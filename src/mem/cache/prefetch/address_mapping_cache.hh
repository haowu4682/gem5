#ifndef __AMC_H
#define __AMC_H

#include "mem/cache/prefetch/logic.hh"

#define AMC_SET_COUNT 128
#define AMC_WAY_COUNT 8
#define SP_AMC_LINE_SIZE 8
#define PS_AMC_LINE_SIZE 8

#define STREAM_MAX_LENGTH 256
#define STREAM_MAX_LENGTH_BITS 8

struct PS_map{
    //uint16_t str_addr:16, valid:1, confidence:2;
    uint32_t str_addr;
    bool valid;
    uint8_t confidence;

    void reset(){
        valid = false;
        str_addr = 0;
        confidence = 0;
    }
    void set(uint32_t addr){
        reset();
        str_addr = addr;
        valid = true;
        confidence = 1;
    }
    void increase_confidence(){
        confidence = (confidence == 3) ? confidence : (confidence+1);
    }
    bool lower_confidence(){
        confidence = (confidence == 0) ? confidence : (confidence-1);
        return confidence;
    }
};


struct PS_AMC_Entry
{
    PS_map map_entries[PS_AMC_LINE_SIZE];

    void reset(){
        for(int i=0; i <PS_AMC_LINE_SIZE; i++)
            map_entries[i].reset();
    }

    void set_str_addr(uint16_t phy_addr, uint32_t str_addr){
        map_entries[phy_addr%PS_AMC_LINE_SIZE].set(str_addr);
    }

    bool get_str_addr(uint16_t phy_addr, uint32_t& str_addr){
        if(map_entries[phy_addr%PS_AMC_LINE_SIZE].valid){
            str_addr = map_entries[phy_addr%PS_AMC_LINE_SIZE].str_addr;
            return true;
        }
        return false;
    }

    bool all_invalidated(){
        for(int i=0; i <PS_AMC_LINE_SIZE; i++){
            if(map_entries[i].valid)
                return false;
        }
        return true;
    }
    
    bool invalidate(uint16_t phy_addr){
        map_entries[phy_addr%PS_AMC_LINE_SIZE].reset();
        return all_invalidated();
    }

    void increase_confidence(uint16_t phy_addr){
        map_entries[phy_addr%PS_AMC_LINE_SIZE].increase_confidence();
    }

    bool lower_confidence(uint16_t phy_addr){
        return (map_entries[phy_addr%PS_AMC_LINE_SIZE].lower_confidence());
    }

    PS_AMC_Entry(){
        reset();
    }
};

struct SP_map{
    uint16_t phy_addr:15, valid:1;

    void reset(){
        valid = false;
        phy_addr = 0;
    }

    void set(uint16_t addr){
        phy_addr = addr;
        valid = true;
    }
};

struct SP_AMC_Entry
{
    SP_map map_entries[SP_AMC_LINE_SIZE];

    void reset(){
        for(int i=0; i <SP_AMC_LINE_SIZE; i++)
            map_entries[i].reset();
    }

    void set_phy_addr(uint16_t phy_addr, uint32_t str_addr){
        map_entries[str_addr%SP_AMC_LINE_SIZE].set(phy_addr);
    }

    bool get_phy_addr(uint16_t& phy_addr, uint32_t str_addr){
        if(map_entries[str_addr%SP_AMC_LINE_SIZE].valid){
            phy_addr = map_entries[str_addr%SP_AMC_LINE_SIZE].phy_addr;
            return true;
        }
        return false;
    }

    bool all_invalidated(){
        for(int i=0; i <SP_AMC_LINE_SIZE; i++){
            if(map_entries[i].valid)
                return false;
        }
        return true;
    }

    bool invalidate(uint32_t str_addr){
        map_entries[str_addr%SP_AMC_LINE_SIZE].reset();
        return all_invalidated();
    }

    SP_AMC_Entry(){
        reset();
    }
};

template<typename T, typename V, int setcount, int waycount, int linesize>
class AMC : public AssociativeArray<T, V, setcount, waycount, linesize>
{
    typedef AssociativeArray<T, V, setcount, waycount, linesize> base_t;

    static int setof(T addr) {
        int set = bits(addr, (log2(linesize) + log2(waycount)), log2(setcount));
        return set;
    }

    public:
    void reset(){
        base_t::reset();
    }

    V* probe(T addr){
        return base_t::sets[setof(addr)].probe(base_t::tagof(addr));
    } 

    V* select(T addr, T& old_addr) {
        //ptl_logfile << " Evicting from set " << setof(addr) << endl;
        return base_t::sets[setof(addr)].select(base_t::tagof(addr), old_addr);
    }

    V* select(T addr) {
        T dummy;
        //ptl_logfile << " Evicting from set " << setof(addr) << endl;
        return base_t::sets[setof(addr)].select(base_t::tagof(addr), dummy);
    }

    int invalidate(T addr) {
        return base_t::sets[setof(addr)].invalidate(base_t::tagof(addr));
    }
};

//Stable Config(AMC) - 256, 8, 8, 256, 8, 8
//Config 1 - 256, 64, 1, 256, 8, 8
//Config 2(AMC) - 16384, 1, 1, 256, 8, 8
class AddressMappingCache
{
//    AssociativeArray<uint32_t, SP_AMC_Entry, AMC_SET_COUNT/4, AMC_WAY_COUNT, SP_AMC_LINE_SIZE> sp_amc;
    AssociativeArray<uint32_t, SP_AMC_Entry, AMC_SET_COUNT, AMC_WAY_COUNT, SP_AMC_LINE_SIZE> sp_amc;
    AMC<uint16_t, PS_AMC_Entry, AMC_SET_COUNT, AMC_WAY_COUNT, PS_AMC_LINE_SIZE> ps_amc;
//    AssociativeArray<uint16_t, PS_AMC_Entry, AMC_SET_COUNT, 2*AMC_WAY_COUNT, PS_AMC_LINE_SIZE> ps_amc;
//    AssociativeArray<uint16_t, PS_AMC_Entry, AMC_SET_COUNT/4, 64, PS_AMC_LINE_SIZE> ps_amc;
    //ISBStats* new_stats;

    public:
    AddressMappingCache(/*ISBStats* new_stats*/);
    void reset();
    bool get_structural_address(uint16_t phy_addr, uint32_t& str_addr);
    bool get_physical_address(uint16_t& phy_addr, uint32_t str_addr);
    void update(uint16_t phy_addr, uint32_t str_addr);
    void invalidate(uint16_t phy_addr, uint32_t str_addr);
    void increase_confidence(uint16_t phy_addr);
    bool lower_confidence(uint16_t phy_addr);
    void reassign_stream(uint32_t str_addr, uint32_t new_str_addr);
};

#endif

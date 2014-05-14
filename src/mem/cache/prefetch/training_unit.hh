#ifndef __MASTER_STREAM_H
#define __MASTER_STREAM_H

#include "mem/cache/prefetch/address_mapping_cache.hh"

#define TU_WAY_COUNT 64
#define PHY_ADDRESS_ENCODER_SIZE 256
#define STR_ADDRESS_ENCODER_SIZE 256

struct TrainingUnitEntry
{
    uint64_t key;
    uint64_t addr;
    uint32_t str_addr;

    TrainingUnitEntry(){
        reset();
    } 
    void reset(){
        key = 0;
        addr = 0;
        str_addr = 0;
    }  
    TrainingUnitEntry(uint64_t _key){
        key = _key;
        addr = 0;
        str_addr = 0;
    }   
};

typedef FullyAssociativeArray<uint64_t, TrainingUnitEntry, TU_WAY_COUNT> TUCache;

template <int ways>
struct AddressEncoderTags : public FullyAssociativeTags<uint64_t, ways>
{
    typedef FullyAssociativeTags<uint64_t, ways> base_t;
    AddressEncoderTags() : base_t() {}

    void reset() {
        base_t::reset();
    }

    bool exists_tag_at_index(int index){
        assert(index >= 0 && index <= ways);
        uint64_t tag = base_t::tags[index];
        return (tag != base_t::INVALID);
    }

    uint64_t get_tag_at_index(int index){
        assert(index >= 0 && index <= ways);
        uint64_t tag = base_t::tags[index];
        assert(tag != base_t::INVALID);
        return tag;
    }

    void set_tag_at_index(int index, uint64_t addr){
        assert(index >= 0 && index <= ways);
        base_t::tags[index] = addr;
        base_t::use(index);
    }
};

struct AddressEncoder
{
    AddressEncoderTags<PHY_ADDRESS_ENCODER_SIZE> phy_addr_encoder;
    AddressEncoderTags<STR_ADDRESS_ENCODER_SIZE> str_addr_encoder;

    public:

    uint64_t decode_phy_addr(uint16_t encoded_addr){
        uint64_t decoded_addr = 0;
        uint16_t page_id = encoded_addr >> 6;
        uint8_t page_offset = encoded_addr%64;
        uint64_t phy_page_addr = phy_addr_encoder.get_tag_at_index(page_id);
        decoded_addr = (phy_page_addr << 12) | (page_offset << 6);
        //ptl_logfile << "Page ID : " << (void*)page_id << " Page addr " << (void*)phy_page_addr << " decoded addr " << (void*)decoded_addr << endl;
        return decoded_addr;
    }

    uint32_t decode_str_addr(uint32_t encoded_str_addr){
        return encoded_str_addr;
    }
/*    uint64_t decode_str_addr(uint16_t encoded_str_addr){
        uint64_t decoded_addr = 0;
        uint16_t page_id = encoded_str_addr >> 8;
        uint8_t page_offset = encoded_str_addr%256;
    //    ptl_logfile << "Decoding str " << (void*)encoded_str_addr << " Page ID : " << (void*)page_id << endl, flush;
        uint64_t str_page_addr = str_addr_encoder.get_tag_at_index(page_id);
        decoded_addr = (str_page_addr << 8) | page_offset;
        return decoded_addr;
    }
*/
    uint16_t encode_phy_addr(uint64_t phy_addr)
    {
        uint64_t encoded_addr = 0;
        uint64_t phy_page_addr = phy_addr >> 12;
        uint8_t page_offset = (phy_addr>>6)%64;

        uint16_t page_id = 0;
        page_id = phy_addr_encoder.probe(phy_page_addr);
        assert(page_id != (uint16_t)(-1));
        encoded_addr = (page_id << 6) | page_offset;
//        ptl_logfile << (void*)phy_addr << " : PHY Page Addr " << (void*)phy_page_addr << " page offset " << (void*)page_offset << " Page ID " << (void*)page_id << " Encoded " << (void*)encoded_addr << endl;
        return encoded_addr;
    }

    bool exists_phy_page(uint64_t phy_addr){
        uint16_t page_id = phy_addr_encoder.probe(phy_addr >> 12);
        if(page_id == (uint16_t)-1)
            return false;
        return true;
    }

    void insert_phy_page(uint64_t page_addr, uint32_t way){
        phy_addr_encoder.set_tag_at_index(way, page_addr);
    }

    bool get_phy_page(uint32_t way, uint64_t& phy_page){
        if(!phy_addr_encoder.exists_tag_at_index(way))
            return false;
        phy_page = phy_addr_encoder.get_tag_at_index(way);
        return true;
    }

    uint32_t encode_str_addr(uint32_t str_addr){
        return str_addr;
    }
/*
    uint16_t encode_str_addr(uint64_t str_addr)
    {
        //Structural addresses are cache line addresses, so we dont need to eliminate the LS 6 bits
        //Also each structural page has STREAM_MAX_LENGTH addresses - 256
        uint16_t encoded_addr = 0;
        uint64_t str_page_addr = 0;
        uint8_t page_offset = 0;
        str_page_addr = str_addr >> 8;
        page_offset = (str_addr % STREAM_MAX_LENGTH);

        uint16_t page_id = str_addr_encoder.probe(str_page_addr);
        if(page_id == (uint16_t)-1){
            page_id = str_addr_encoder.select(str_page_addr);
            ptl_logfile << " Evicted Str Page id " << (void*)page_id << endl;
        }

        ptl_logfile << "Addr " << (void*)str_addr << " STR Page Addr " << (void*)str_page_addr << " page offset " << (void*)page_offset << " Page ID " << (void*)page_id << endl;
        encoded_addr = (page_id << 8) | page_offset;
        return encoded_addr;
    }
*/
    void reset(){
        phy_addr_encoder.reset();
        str_addr_encoder.reset();
    }
};


#endif

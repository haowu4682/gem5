#ifndef __OFFCHIP_H__
#define __OFFCHIP_H__

#include "mem/cache/prefetch/logic.hh"

struct CorrMatrixEntry : public selflistlink {
    uint64_t key_addr;
    uint64_t neighbor[64];
    uint64_t str_addr[64];
    uint8_t confidence[64];
    bool neighbor_found[64];
    bool str_addr_present[64];

    void init(uint64_t phy_addr_){
        key_addr = phy_addr_;
        for(int i=0; i<64; i++){
            neighbor_found[i] = false;
            str_addr_present[i] = false;
            confidence[i] = 0;
        }
    }

    void update_neighbor(uint64_t neighbor_, uint32_t offset){
        if(!neighbor_found[offset]){
            neighbor[offset] = neighbor_;
            confidence[offset] = 1;
            neighbor_found[offset] = true;
            return;
        }
        
        if(neighbor[offset] == neighbor_ ){
            confidence[offset] = (confidence[offset]>=3) ? confidence[offset] : confidence[offset]+1;
            return;
        }
        confidence[offset]--;
        if(confidence[offset])
            return;
        neighbor[offset] = neighbor_;
        confidence[offset] = 1;
    }

    void update_str_addr(uint64_t str_addr_[64], bool mask[64])
    {
/*
        for(int i=0; i<64; i++){
            if(mask[i]){
                str_addr[i] = str_addr_[i];
                str_addr_present[i] = true;
            }
        }
*/
        memcpy(str_addr, str_addr_, 64*sizeof(uint64_t));
        memcpy(str_addr_present, mask, 64*sizeof(bool));
    }
};

struct CorrMatrixLinkManager: public ObjectLinkManager<CorrMatrixEntry> {
  static inline uint64_t& keyof(CorrMatrixEntry* obj) {
    return obj->key_addr;
  }
};

struct CorrMatrixType : public SelfHashtable<uint64_t, CorrMatrixEntry, 64, CorrMatrixLinkManager>
{
    typedef SelfHashtable<uint64_t, CorrMatrixEntry, 64, CorrMatrixLinkManager> base_t;

    void update_neighbor(uint64_t phy_addr, uint64_t neighbor){
        CorrMatrixEntry* off_chip_entry = base_t::get(phy_addr>>12);  
        if(off_chip_entry == NULL){
            CorrMatrixEntry* info = (CorrMatrixEntry*)malloc(sizeof(CorrMatrixEntry));
            assert(info);
            off_chip_entry = info;
            off_chip_entry->init(phy_addr>>12);
            off_chip_entry->update_neighbor(neighbor, ((phy_addr>>6)%64)); 
            base_t::add(off_chip_entry);
        }
        else{
            off_chip_entry->update_neighbor(neighbor, ((phy_addr>>6)%64)); 
        }
        assert(off_chip_entry);
    }

//    void update_str_addr(uint64_t phy_addr, uint64_t str_addr){
    uint64_t update_str_addr(uint64_t phy_page_addr, uint64_t str_addr[64], bool present[64]){
        CorrMatrixEntry* off_chip_entry = base_t::get(phy_page_addr);  
        if(off_chip_entry == NULL){
            CorrMatrixEntry* info = (CorrMatrixEntry*)malloc(sizeof(CorrMatrixEntry));
            assert(info);
            off_chip_entry = info;
            off_chip_entry->init(phy_page_addr);
            off_chip_entry->update_str_addr(str_addr, present);
            base_t::add(off_chip_entry);
        }
        else{
            off_chip_entry->update_str_addr(str_addr, present); 
        }
        assert(off_chip_entry);
        return ((uint64_t)off_chip_entry);
    }

    bool get_neighbor(uint64_t phy_addr, uint64_t& neighbor){
        CorrMatrixEntry* off_chip_entry = base_t::get(phy_addr>>12);  
        if(off_chip_entry == NULL)
            return false;
        uint32_t offset = (phy_addr>>6) % 64;
        if(!(off_chip_entry->neighbor_found[offset]))
            return false;
        neighbor = off_chip_entry->neighbor[offset];
        return true;
    }

    uint64_t get_structural_address(uint64_t phy_page_addr, uint64_t str_addr[64], bool present[64])
    {
        CorrMatrixEntry* off_chip_entry = base_t::get(phy_page_addr);  
        if(off_chip_entry == NULL)
            return 0;
        //  str_addr[i] = off_chip_entry->str_addr[i];
        //  present[i] = off_chip_entry->str_addr_present[i];
        memcpy(str_addr, off_chip_entry->str_addr, 64*sizeof(uint64_t));
        memcpy(present, off_chip_entry->str_addr_present, 64*sizeof(bool));
        
        return ((uint64_t)off_chip_entry);
    }
};
#endif

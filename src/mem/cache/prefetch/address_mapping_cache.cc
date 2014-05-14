#include "mem/cache/prefetch/address_mapping_cache.hh"

AddressMappingCache::AddressMappingCache(/*ISBStats* _new_stats*/)
{
    reset();
    //new_stats = _new_stats;
}

void AddressMappingCache::reset()
{
    ps_amc.reset();
    sp_amc.reset();
}

bool AddressMappingCache::get_structural_address(W16 phy_addr, W32& str_addr)
{
    N_STAT_UPDATE((*new_stats).ps_amc_read, ++, 0);
    PS_AMC_Entry* entry = ps_amc.probe(phy_addr);
    if(entry){
        bool ret = entry->get_str_addr(phy_addr, str_addr);
        return ret;
    }
    return false;
}

bool AddressMappingCache::get_physical_address(W16& phy_addr, W32 str_addr)
{
    N_STAT_UPDATE((*new_stats).sp_amc_read, ++, 0);
    SP_AMC_Entry* entry = sp_amc.probe(str_addr);
    if(entry){
        bool ret = entry->get_phy_addr(phy_addr, str_addr);
        return ret;
    }
    return false;
}

void AddressMappingCache::update(W16 phy_addr, W32 str_addr)
{
    N_STAT_UPDATE((*new_stats).ps_amc_write, ++, 0);
    PS_AMC_Entry* ps_entry = ps_amc.probe(phy_addr);
    if(!ps_entry){
        ps_entry = ps_amc.select(phy_addr);
        ps_entry->reset();
    }
    ps_entry->set_str_addr(phy_addr, str_addr);

    N_STAT_UPDATE((*new_stats).sp_amc_write, ++, 0);
    SP_AMC_Entry* sp_entry = sp_amc.probe(str_addr);
    if(!sp_entry){
        W32 old_addr;
        sp_entry = sp_amc.select(str_addr, old_addr);
        //ptl_logfile << "SP Entry evicted " << (void*)old_addr << endl;
        sp_entry->reset();
    }
    sp_entry->set_phy_addr(phy_addr, str_addr);
/*
    PS_AMC_Entry* ps_entry = ps_amc.select(phy_addr);
    ps_entry->reset();
    ps_entry->set_str_addr(phy_addr, str_addr);
    SP_AMC_Entry* sp_entry = sp_amc.select(str_addr);
    sp_entry->reset();
    sp_entry->set_phy_addr(phy_addr, str_addr);
*/
}

void AddressMappingCache::invalidate(W16 phy_addr, W32 str_addr)
{
    N_STAT_UPDATE((*new_stats).sp_amc_write, ++, 0);
    N_STAT_UPDATE((*new_stats).ps_amc_write, ++, 0);
    PS_AMC_Entry* ps_entry = ps_amc.probe(phy_addr);
    assert(ps_entry);
    if(ps_entry->invalidate(phy_addr)){
        ps_amc.invalidate(phy_addr);
    }
    SP_AMC_Entry* sp_entry = sp_amc.probe(str_addr);
    if(sp_entry){
        if(sp_entry->invalidate(str_addr)){
            sp_amc.invalidate(str_addr);
        }
    }
}

void AddressMappingCache::increase_confidence(W16 phy_addr)
{
    N_STAT_UPDATE((*new_stats).ps_amc_write, ++, 0);
    PS_AMC_Entry* ps_entry = ps_amc.probe(phy_addr);
    assert(ps_entry);
    ps_entry->increase_confidence(phy_addr);
}

bool AddressMappingCache::lower_confidence(W16 phy_addr)
{
    N_STAT_UPDATE((*new_stats).ps_amc_write, ++, 0);
    PS_AMC_Entry* ps_entry = ps_amc.probe(phy_addr);
    assert(ps_entry);
    bool ret = ps_entry->lower_confidence(phy_addr);
    return ret;
}

void AddressMappingCache::reassign_stream(W32 str_addr, W32 new_str_addr)
{
    W32 addr = str_addr;
    while(addr%STREAM_MAX_LENGTH != 0)
    {
        //sp_read
        SP_AMC_Entry* sp_entry = sp_amc.probe(addr);
        W16 phy_addr;
        if(sp_entry && sp_entry->get_phy_addr(phy_addr, addr)){
//            ptl_logfile << " Reassigning " << (void*)phy_addr << " from " << (void*)addr << " to " << (void*)new_str_addr << endl, flush;

            if(sp_entry->invalidate(addr))
                sp_amc.invalidate(addr);

            PS_AMC_Entry* ps_entry = ps_amc.probe(phy_addr);
            // TODO - assert ps_entry
            if(ps_entry){
                ps_entry->set_str_addr(phy_addr, new_str_addr);
                N_STAT_UPDATE((*new_stats).ps_amc_write, ++, 0);
            }
       
            SP_AMC_Entry* new_sp_entry = sp_amc.probe(new_str_addr);
            if(!new_sp_entry){
                W32 old_addr;
                new_sp_entry = sp_amc.select(new_str_addr, old_addr);
                //ptl_logfile << "SP Entry evicted " << (void*)old_addr << endl;
                new_sp_entry->reset();
            }
            N_STAT_UPDATE((*new_stats).sp_amc_write, ++, 0);
            new_sp_entry->set_phy_addr(phy_addr, new_str_addr);
/*
            SP_AMC_Entry* new_sp_entry = sp_amc.select(new_str_addr);
            new_sp_entry->reset();
            new_sp_entry->set_phy_addr(phy_addr, new_str_addr);
*/
        }
        addr++;
        new_str_addr++;
    }
}

//Irregular Stream Buffer - Implementation by Akanksha Jain

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/isb.hh"

//#define DEBUG

// Arguments passed are virtual page tags so the last 12 bits have already been dropped
void ISBPrefetcher::inform_tlb_eviction(uint64_t inserted_page_addr, uint32_t way)
{
    //if(inserted_page_addr == last_page)
    if((inserted_page_addr == last_page) || addr_encoder.exists_phy_page(inserted_page_addr<<12))
        return;
    int count = 0;
    // Invalidate this page in AMC and update the off-chip matrix
    uint64_t str_addr_to_write[64];
    bool present[64];
    uint64_t phy_page_to_evict;
    if(addr_encoder.get_phy_page(way, phy_page_to_evict)){

        N_STAT_UPDATE(new_stats.pages_written, ++, 0);
        for(uint8_t i=0; i<64; i++)
            present[i] = false;

        for(uint8_t i=0; i<64; i++){
            uint16_t phy_addr_to_evict = (way << 6) | i;
            uint32_t str_addr_to_evict;
            if(amc.get_structural_address(phy_addr_to_evict, str_addr_to_evict)){
                str_addr_to_write[i] = addr_encoder.decode_str_addr(str_addr_to_evict); 
                present[i] = true;
                amc.invalidate(phy_addr_to_evict, str_addr_to_evict);
                count++;
            }
        }
        uint64_t write_meta_data_addr = off_chip_corr_matrix.update_str_addr(phy_page_to_evict, str_addr_to_write, present);
        InitMetaDataWrite(write_meta_data_addr);
    }
#ifdef DEBUG
    ptl_logfile << "Inserting " << (void*)inserted_page_addr <<  " into way " << (void*)way << endl, flush;
    ptl_logfile << "Evicted " << (void*)phy_page_to_evict <<  " into way " << (void*)way << endl, flush;
#endif
    addr_encoder.insert_phy_page(inserted_page_addr, way);
    assert(count <= 64);
    //phy_page_entry_used[count]++; 
    N_STAT_UPDATE(new_stats.page_entries_mapped, [count]++, 0);
    N_STAT_UPDATE(new_stats.pages_read, ++, 0);

    // Fetch this page from the off-chip matrix
    uint64_t str_addr_to_fetch[64];
    for(int i=0; i<64; i++)
        present[i] = false;

    uint64_t read_meta_data_addr = off_chip_corr_matrix.get_structural_address(inserted_page_addr, str_addr_to_fetch, present);
    if(read_meta_data_addr){

        InitMetaDataRead(read_meta_data_addr);

        for(uint8_t i=0; i<64; i++){
            if(present[i]){
                uint64_t phy_addr_to_fetch = (inserted_page_addr << 12) | (i << 6);
                amc.update(addr_encoder.encode_phy_addr(phy_addr_to_fetch), addr_encoder.encode_str_addr(str_addr_to_fetch[i]));
            }
        }
    }

    last_page = inserted_page_addr;
//    ptl_logfile << "T : " << count << endl;
/*
    ptl_logfile << "Divergence count " << stream_divergence_count << endl;
    ptl_logfile << "Candidate TLB Miss " << candidate_tlb_miss << endl;
    ptl_logfile << "Candidate diff " << candidate_diff << endl;
    ptl_logfile << "Phy Page Entry Count"  << endl;
    for(int i=0; i<=64; i++)
        ptl_logfile << phy_page_entry_used[i] << " ";
    ptl_logfile << endl;
*/
    for(uint8_t i=0; i<BUFFER_SIZE; i++){
        uint16_t phy_addr;
        if(!prefetch_buffer.valid[i])
            continue;
        bool ret = amc.get_physical_address(phy_addr, prefetch_buffer.buffer[i]);
        if(ret){
            uint64_t candidate = addr_encoder.decode_phy_addr(phy_addr);
        //    ptl_logfile << " Buffer Success " << (void*)(prefetch_buffer.buffer[i]) << " " << (void*)candidate << endl;
            InitPrefetch(candidate);
            prefetch_buffer.issue(i);
        }
    }
}

uint32_t ISBPrefetcher::isb_train(uint32_t str_addr_A, uint16_t encoded_phy_addr_B)
{
    //Algorithm for training correlated pair (A,B) 
    //Step 2a : If SA(A)+1 does not exist, assign B SA(A)+1
    //Step 2b : If SA(A)+1 exists, copy the stream starting at S(A)+1 and then assign B SA(A)+1
    //ptl_logfile << "Start " << endl;
    uint32_t str_addr_B;
    bool str_addr_B_exists = amc.get_structural_address(encoded_phy_addr_B, str_addr_B); 
#ifdef DEBUG
    ptl_logfile << "-----S(A) : " << (void*)str_addr_A << endl;
#endif
    // If S(A) is at a stream boundary return, we don't need to worry about B because it is as good as a stream start
    if((str_addr_A+1) % STREAM_MAX_LENGTH == 0){
        if(!str_addr_B_exists){
            str_addr_B = assign_structural_addr();
            amc.update(encoded_phy_addr_B, str_addr_B);
        }
        return str_addr_B;
    }

    if(str_addr_B_exists){
        if(str_addr_B == str_addr_A + 1){
#ifdef DEBUG
            ptl_logfile << (void*)encoded_phy_addr_B << " has a structural address of " << (void*)str_addr_B << " conf++ " << endl;
#endif
            amc.increase_confidence(encoded_phy_addr_B);
            return str_addr_B;
        }
/*
        else if(((str_addr_B >> STREAM_MAX_LENGTH_BITS ) == (str_addr_A >> STREAM_MAX_LENGTH_BITS )) && (str_addr_B < str_addr_A)){
#ifdef DEBUG
            ptl_logfile << (void*)encoded_phy_addr_B << " has a structural address of " << (void*)str_addr_B << " conf no change " << endl;
#endif
            return str_addr_B;
        }
*/
        else{
#ifdef DEBUG
            ptl_logfile << (void*)encoded_phy_addr_B << " has a structural address of " << (void*)str_addr_B << " conf-- " << endl;
#endif
            bool ret = amc.lower_confidence(encoded_phy_addr_B);
            if(ret)
                return str_addr_B;
#ifdef DEBUG
            ptl_logfile << "Invalidate " << endl, flush;
#endif
            amc.invalidate(encoded_phy_addr_B, str_addr_B);
            str_addr_B_exists = false;
        }
    }

    assert(!str_addr_B_exists);

    //Handle stream divergence
    uint16_t encoded_phy_addr_Aplus1;
    bool encoded_phy_addr_Aplus1_exists = amc.get_physical_address(encoded_phy_addr_Aplus1, str_addr_A+1);
    if(encoded_phy_addr_Aplus1_exists){
#ifdef DEBUG
        ptl_logfile << "-----S(A)+1 : " << (void*)encoded_phy_addr_Aplus1 << endl;
#endif
        stream_divergence_count++;
        amc.reassign_stream(str_addr_A+1, assign_structural_addr());
        N_STAT_UPDATE(new_stats.reassigned, ++, 0);
    }

    str_addr_B = str_addr_A + 1;
    N_STAT_UPDATE(new_stats.alloc_stream_length, [(str_addr_B%256)]++, 0);
#ifdef DEBUG
    ptl_logfile << (void*)encoded_phy_addr_B << " allotted a structural address of " << (void*)str_addr_B << endl;
    ptl_logfile << "-----S(B) : " << (void*)str_addr_B << endl;
#endif
    amc.update(encoded_phy_addr_B, str_addr_B);

    return str_addr_B;
}

uint32_t ISBPrefetcher::assign_structural_addr()
{
    alloc_counter += STREAM_MAX_LENGTH;
    N_STAT_UPDATE(new_stats.stream_length, [0]++, 0);
    N_STAT_UPDATE(new_stats.alloc_stream_length, [0]++, 0);
#ifdef DEBUG
    ptl_logfile << "  ALLOC " << (void*)alloc_counter << endl;
#endif
    return addr_encoder.encode_str_addr(alloc_counter);
}

bool ISBPrefetcher::access_training_unit(uint64_t key, uint64_t& last_phy_addr, uint32_t& last_str_addr, uint64_t next_addr)
{
    TrainingUnitEntry* curr_training_entry = training_unit.probe(key);
    bool pair_found = true;
    if(curr_training_entry == NULL)
    {
        TrainingUnitEntry* new_training_entry = training_unit.select(key);
        assert(new_training_entry);
        new_training_entry->reset();
        curr_training_entry = new_training_entry;
        pair_found = false;
    }

    assert(curr_training_entry != NULL);
    last_str_addr = curr_training_entry->str_addr;
    last_phy_addr = curr_training_entry->addr;
    uint64_t last_addr = curr_training_entry->addr;
    if(last_addr == next_addr)
        return false;
#ifdef DEBUG
    off_chip_corr_matrix.update_neighbor(last_addr, next_addr);
#endif
    return pair_found;
}

void ISBPrefetcher::update_training_unit(uint64_t key, uint64_t addr, uint32_t str_addr)
{
    TrainingUnitEntry* curr_training_entry = training_unit.probe(key);
    assert(curr_training_entry);
    curr_training_entry->addr = addr;
    curr_training_entry->str_addr = str_addr;
}

void ISBPrefetcher::isb_predict(uint64_t trigger_phy_addr, uint32_t trigger_str_addr)
{
#ifdef DEBUG
    ptl_logfile << "*Trigger Str addr " << (void*)trigger_str_addr << endl, flush;
#endif
    uint64_t candidate;
    uint16_t phy_addr;

    for(uint32_t i=0; i<degree; i++)
    {
        //bool ret = amc.get_physical_address(phy_addr, trigger_str_addr+lookahead);
        uint64_t str_addr_candidate = trigger_str_addr+lookahead+i ;
        if(str_addr_candidate % STREAM_MAX_LENGTH == 0)
            return;
        bool ret = amc.get_physical_address(phy_addr, str_addr_candidate);
        if(ret){
            candidate = addr_encoder.decode_phy_addr(phy_addr);
#ifdef DEBUG
            ptl_logfile << "Prefetching " << (void*)candidate << endl;
#endif
            InitPrefetch(candidate, 2*i); 
            //uint32_t sl = ((str_addr_candidate) % 256);
            N_STAT_UPDATE(new_stats.stream_length, [sl]++, 0);
            //    N_STAT_UPDATE(new_stats.stream_length, [sl-1]--, 0);
        }
        else{
            prefetch_buffer.add(str_addr_candidate);
        }
    }

#ifdef DEBUG
    uint64_t m_candidate;
    if(off_chip_corr_matrix.get_neighbor(trigger_phy_addr, m_candidate)){
 //       InitPrefetch(m_candidate); 
        if(!addr_encoder.exists_phy_page(m_candidate))
        {
         //   ptl_logfile << " Candidate outside page boundary " << (void*) m_candidate << endl;
            candidate_tlb_miss++;
        }
        else{
          //  InitPrefetch(m_candidate); 
            if(!ret){
          //      ptl_logfile << "ISB no pred " << (void*) m_candidate << endl;
            }
            if(ret && m_candidate == candidate){
            //    ptl_logfile << "Agree " << endl;
            }
            if(ret && m_candidate != candidate){
                candidate_diff++;
            //    ptl_logfile << " Disagree " << (void*) m_candidate << endl;
            }
        }
    }
#endif
}

void ISBPrefetcher::IssuePrefetchCandidates(uint64_t key, uint64_t addr_B, bool mshr_hit, bool hit )
{
     //key = 0;
    if(!addr_encoder.exists_phy_page(addr_B))
        return;
#ifdef DEBUG
    ptl_logfile << "**Trigger " << (void*)addr_B << " with key " << (void*)key << endl, flush;
#endif
    uint16_t encoded_phy_addr_B = addr_encoder.encode_phy_addr(addr_B);
#ifdef DEBUG
    ptl_logfile << "Encoded Trigger " << (void*)encoded_phy_addr_B << endl;
#endif
    uint32_t str_addr_B = 0;
    bool str_addr_B_exists = amc.get_structural_address(encoded_phy_addr_B, str_addr_B); 
    if(str_addr_B_exists){
        isb_predict(addr_B, str_addr_B);
    }

    uint32_t str_addr_A;
    uint64_t addr_A;
    if(access_training_unit(key, addr_A, str_addr_A, addr_B)){
#ifdef DEBUG
        ptl_logfile << "        Consider pair " << (void*)str_addr_A << " and " <<(void*)encoded_phy_addr_B << " with key as " << (void*) key << endl;
#endif
        if(str_addr_A == 0 && addr_encoder.exists_phy_page(addr_A)){
            uint16_t encoded_phy_addr_A = addr_encoder.encode_phy_addr(addr_A);
            str_addr_A = assign_structural_addr();
            amc.update(encoded_phy_addr_A, str_addr_A);
        }
        str_addr_B = isb_train(str_addr_A, encoded_phy_addr_B);
    }
    /*else if(str_addr_A == 0){
        str_addr_B = assign_structural_addr();
        //amc.update(encoded_phy_addr_B, str_addr_B);
#ifdef DEBUG
        ptl_logfile << (void*)encoded_phy_addr_B << " allotted a structural address of " << (void*)str_addr_B << endl;
#endif
    }
    */

    //sb.update_state(str_addr_B);
    update_training_unit(key, addr_B, str_addr_B);
    //sb.print(str_addr_B);
}

bool ISBPrefetcher::InitMetaDataWrite(Addr addr, int delay)
{
    bool is_secure = false;

    // create a prefetch memreq
    Request *prefetchReq = new Request(addr, blkSize, 0, masterId);
    if (is_secure)
        prefetchReq->setFlags(Request::SECURE);
    prefetchReq->taskId(ContextSwitchTaskId::Prefetcher);
    PacketPtr prefetch =
        new Packet(prefetchReq, MemCmd::WriteReq);
    prefetch->allocate();
    prefetch->req->setThreadContext(contextId, threadId);

    // We just remove the head if we are full
    if (pf.size() == size) {
        pfRemovedFull++;
        PacketPtr old_pkt = pf.begin()->pkt;
        DPRINTF(HWPrefetch, "Prefetch queue full, "
                "removing oldest 0x%x\n", old_pkt->getAddr());
        delete old_pkt->req;
        delete old_pkt;
        pf.pop_front();
    }

    pf.push_back(DeferredPacket(current_prefetch_tick + clockPeriod() * delay, prefetch));

    return true;
}

bool ISBPrefetcher::InitMetaDataRead(Addr addr, int delay)
{
    bool is_secure = false;

    // create a prefetch memreq
    Request *prefetchReq = new Request(addr, blkSize, 0, masterId);
    if (is_secure)
        prefetchReq->setFlags(Request::SECURE);
    prefetchReq->taskId(ContextSwitchTaskId::Prefetcher);
    PacketPtr prefetch =
        new Packet(prefetchReq, MemCmd::ReadReq);
    prefetch->allocate();
    prefetch->req->setThreadContext(contextId, threadId);

    // We just remove the head if we are full
    if (pf.size() == size) {
        pfRemovedFull++;
        PacketPtr old_pkt = pf.begin()->pkt;
        DPRINTF(HWPrefetch, "Prefetch queue full, "
                "removing oldest 0x%x\n", old_pkt->getAddr());
        delete old_pkt->req;
        delete old_pkt;
        pf.pop_front();
    }

    pf.push_back(DeferredPacket(current_prefetch_tick + clockPeriod() * delay, prefetch));

    return true;
}

ISBPrefetcher*
ISBPrefetcherParams::create()
{
   return new ISBPrefetcher(this);
}


/*
 * Copyright (c) 2010-2012, 2014-2016 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_RENAME_IMPL_HH__
#define __CPU_O3_RENAME_IMPL_HH__

#include <list>

#include "arch/isa_traits.hh"
#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "cpu/o3/rename.hh"
#include "cpu/reg_class.hh"
#include "debug/Activity.hh"
#include "debug/Rename.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template <class Impl>
DefaultRename<Impl>::DefaultRename(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      iewToRenameDelay(params->iewToRenameDelay),
      decodeToRenameDelay(params->decodeToRenameDelay),
      commitToRenameDelay(params->commitToRenameDelay),
      renameWidth(params->renameWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numThreads)
{
    if (renameWidth > Impl::MaxWidth)
        fatal("renameWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             renameWidth, static_cast<int>(Impl::MaxWidth));

    // @todo: Make into a parameter.
    skidBufferMax = (decodeToRenameDelay + 1) * params->decodeWidth;
    LTPMax = 128;
    for (uint32_t tid = 0; tid < Impl::MaxThreads; tid++) {
        renameStatus[tid] = Idle;
        renameMap[tid] = nullptr;
        toIQInProgress[tid] = 0;
        toRobInProgress[tid] = 0;
        loadsInProgress[tid] = 0;
        storesInProgress[tid] = 0;
        freeEntries[tid] = {0, 0, 0, 0};
        emptyROB[tid] = true;
        stalls[tid] = {false, false};
        serializeInst[tid] = nullptr;
        serializeOnNextInst[tid] = false;
        gateLTP[tid] = false;
    }
    maxSecRename = 8;
    instsWakeNum = 0;

}

template <class Impl>
std::string
DefaultRename<Impl>::name() const
{
    return cpu->name() + ".rename";
}

template <class Impl>
void
DefaultRename<Impl>::regStats()
{
    renameSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles rename is squashing")
        .prereq(renameSquashCycles);
    renameIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles rename is idle")
        .prereq(renameIdleCycles);
    renameBlockCycles
        .name(name() + ".BlockCycles")
        .desc("Number of cycles rename is blocking")
        .prereq(renameBlockCycles);
    renameSerializeStallCycles
        .name(name() + ".serializeStallCycles")
        .desc("count of cycles rename stalled for serializing inst")
        .flags(Stats::total);
    renameRunCycles
        .name(name() + ".RunCycles")
        .desc("Number of cycles rename is running")
        .prereq(renameIdleCycles);
    renameUnblockCycles
        .name(name() + ".UnblockCycles")
        .desc("Number of cycles rename is unblocking")
        .prereq(renameUnblockCycles);
    renameRenamedInsts
        .name(name() + ".RenamedInsts")
        .desc("Number of instructions processed by rename")
        .prereq(renameRenamedInsts);
    renameSquashedInsts
        .name(name() + ".SquashedInsts")
        .desc("Number of squashed instructions processed by rename")
        .prereq(renameSquashedInsts);
    renameROBFullEvents
        .name(name() + ".ROBFullEvents")
        .desc("Number of times rename has blocked due to ROB full")
        .prereq(renameROBFullEvents);
    renameIQFullEvents
        .name(name() + ".IQFullEvents")
        .desc("Number of times rename has blocked due to IQ full")
        .prereq(renameIQFullEvents);
    renameLQFullEvents
        .name(name() + ".LQFullEvents")
        .desc("Number of times rename has blocked due to LQ full")
        .prereq(renameLQFullEvents);
    renameSQFullEvents
        .name(name() + ".SQFullEvents")
        .desc("Number of times rename has blocked due to SQ full")
        .prereq(renameSQFullEvents);
    renameFullRegistersEvents
        .name(name() + ".FullRegisterEvents")
        .desc("Number of times there has been no free registers")
        .prereq(renameFullRegistersEvents);
    renameRenamedOperands
        .name(name() + ".RenamedOperands")
        .desc("Number of destination operands rename has renamed")
        .prereq(renameRenamedOperands);
    renameRenameLookups
        .name(name() + ".RenameLookups")
        .desc("Number of register rename lookups that rename has made")
        .prereq(renameRenameLookups);
    renameCommittedMaps
        .name(name() + ".CommittedMaps")
        .desc("Number of HB maps that are committed")
        .prereq(renameCommittedMaps);
    renameUndoneMaps
        .name(name() + ".UndoneMaps")
        .desc("Number of HB maps that are undone due to squashing")
        .prereq(renameUndoneMaps);
    renamedSerializing
        .name(name() + ".serializingInsts")
        .desc("count of serializing insts renamed")
        .flags(Stats::total)
        ;
    renamedTempSerializing
        .name(name() + ".tempSerializingInsts")
        .desc("count of temporary serializing insts renamed")
        .flags(Stats::total)
        ;
    renameSkidInsts
        .name(name() + ".skidInsts")
        .desc("count of insts added to the skid buffer")
        .flags(Stats::total)
        ;
    intRenameLookups
        .name(name() + ".int_rename_lookups")
        .desc("Number of integer rename lookups")
        .prereq(intRenameLookups);
    fpRenameLookups
        .name(name() + ".fp_rename_lookups")
        .desc("Number of floating rename lookups")
        .prereq(fpRenameLookups);
    vecRenameLookups
        .name(name() + ".vec_rename_lookups")
        .desc("Number of vector rename lookups")
        .prereq(vecRenameLookups);
    vecPredRenameLookups
        .name(name() + ".vec_pred_rename_lookups")
        .desc("Number of vector predicate rename lookups")
        .prereq(vecPredRenameLookups);
    InLTPInsts
        .name(name() + ".InLTPInsts")
        .desc("Number of insts insert into LTP")
        .prereq(InLTPInsts);
    numLTPInstsPer
        .init(0,renameWidth,1)
        .name(name() + ".insert_in_LTP_per_cycle")
        .desc("Number of insts insert into LTP per cycle")
        .flags(Stats::pdf)
        ;
    closeLTPCycles
        .name(name() + ".LTP_close_num_of_cycles")
        .desc("Number of cycles LTP close")
        .prereq(closeLTPCycles);
    openLTPCycles
        .name(name() + ".LTP_open_num_of_cycles")
        .desc("Number of cycles LTP open")
        .prereq(openLTPCycles);

}

template <class Impl>
void
DefaultRename<Impl>::regProbePoints()
{
    ppRename = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Rename");
    ppSquashInRename = new ProbePointArg<SeqNumRegPair>(cpu->getProbeManager(),
                                                        "SquashInRename");
}

template <class Impl>
void
DefaultRename<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from IEW stage.
    fromIEW = timeBuffer->getWire(-iewToRenameDelay);

    // Setup wire to read infromation from time buffer, from commit stage.
    fromCommit = timeBuffer->getWire(-commitToRenameDelay);

    // Setup wire to write information to previous stages.
    toDecode = timeBuffer->getWire(0);
}

template <class Impl>
void
DefaultRename<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr_iew,TimeBuffer<RenameCommitStruct> *rq_ptr_commit)
{
    renameQueueIEW = rq_ptr_iew;
    renameQueueCommit = rq_ptr_commit;

    // Setup wire to write information to future stages.
    toIEWW = renameQueueIEW->getWire(0);
    toCommit = renameQueueCommit->getWire(0);

}

template <class Impl>
void
DefaultRename<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    decodeQueue = dq_ptr;

    // Setup wire to get information from decode.
    fromDecode = decodeQueue->getWire(-decodeToRenameDelay);
}

template <class Impl>
void
DefaultRename<Impl>::startupStage()
{
    resetStage();
}

template <class Impl>
void
DefaultRename<Impl>::clearStates(ThreadID tid)
{
    renameStatus[tid] = Idle;

    freeEntries[tid].iqEntries = iew_ptr->instQueue.numFreeEntries(tid);
    freeEntries[tid].lqEntries = iew_ptr->ldstQueue.numFreeLoadEntries(tid);
    freeEntries[tid].sqEntries = iew_ptr->ldstQueue.numFreeStoreEntries(tid);
    freeEntries[tid].robEntries = commit_ptr->numROBFreeEntries(tid);
    emptyROB[tid] = true;

    stalls[tid].iew = false;
    serializeInst[tid] = NULL;
    
    toRobInProgress[tid] = 0;
    toIQInProgress[tid] = 0;
    loadsInProgress[tid] = 0;
    storesInProgress[tid] = 0;

    serializeOnNextInst[tid] = false;
    gateLTP[tid] = false;
    
}

template <class Impl>
void
DefaultRename<Impl>::resetStage()
{
    _status = Inactive;

    resumeSerialize = false;
    resumeUnblocking = false;

    // Grab the number of free entries directly from the stages.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        renameStatus[tid] = Idle;

        freeEntries[tid].iqEntries = iew_ptr->instQueue.numFreeEntries(tid);
        freeEntries[tid].lqEntries = iew_ptr->ldstQueue.numFreeLoadEntries(tid);
        freeEntries[tid].sqEntries = iew_ptr->ldstQueue.numFreeStoreEntries(tid);
        freeEntries[tid].robEntries = commit_ptr->numROBFreeEntries(tid);
        emptyROB[tid] = true;

        stalls[tid].iew = false;
        serializeInst[tid] = NULL;

        toIQInProgress[tid] = 0;
        toRobInProgress[tid] = 0;
        loadsInProgress[tid] = 0;
        storesInProgress[tid] = 0;

        serializeOnNextInst[tid] = false;
    }
}

template<class Impl>
void
DefaultRename<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}


template <class Impl>
void
DefaultRename<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    for (ThreadID tid = 0; tid < numThreads; tid++)
        renameMap[tid] = &rm_ptr[tid];
}

template <class Impl>
void
DefaultRename<Impl>::setFreeList(FreeList *fl_ptr)
{
    freeList = fl_ptr;
}

template<class Impl>
void
DefaultRename<Impl>::setScoreboard(Scoreboard *_scoreboard)
{
    scoreboard = _scoreboard;
}

template <class Impl>
bool
DefaultRename<Impl>::isDrained() const
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (toIQInProgress[tid] != 0 ||
            !historyBufferExt[tid].empty() ||
            !historyBufferSec[tid].empty() ||
            !skidBuffer[tid].empty() ||
            !insts[tid].empty() ||
            (renameStatus[tid] != Idle && renameStatus[tid] != Running))
            return false;
    }
    return true;
}

template <class Impl>
void
DefaultRename<Impl>::takeOverFrom()
{
    resetStage();
}

template <class Impl>
void
DefaultRename<Impl>::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        assert(historyBufferSec[tid].empty());
        assert(historyBufferExt[tid].empty());
        assert(insts[tid].empty());
        assert(skidBuffer[tid].empty());
        assert(toIQInProgress[tid] == 0);
        assert(toRobInProgress[tid] == 0);
    }
}

template <class Impl>
void
DefaultRename<Impl>::squash(const InstSeqNum &squash_seq_num, ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Squashing instructions.\n",tid);

    // Clear the stall signal if rename was blocked or unblocking before.
    // If it still needs to block, the blocking should happen the next
    // cycle and there should be space to hold everything due to the squash.
    if (renameStatus[tid] == Blocked ||
        renameStatus[tid] == Unblocking) {
        toDecode->renameUnblock[tid] = 1;

        resumeSerialize = false;
        serializeInst[tid] = NULL;
    } else if (renameStatus[tid] == SerializeStall) {
        if (serializeInst[tid]->seqNum <= squash_seq_num) {
            DPRINTF(Rename, "Rename will resume serializing after squash\n");
            resumeSerialize = true;
            assert(serializeInst[tid]);
        } else {
            resumeSerialize = false;
            toDecode->renameUnblock[tid] = 1;

            serializeInst[tid] = NULL;
        }
    }

    // Set the status to Squashing.
    renameStatus[tid] = Squashing;

    // Squash any instructions from decode.
    for (int i=0; i<fromDecode->size; i++) {
        if (fromDecode->insts[i]->threadNumber == tid &&
            fromDecode->insts[i]->seqNum > squash_seq_num) {
            fromDecode->insts[i]->setSquashed();
            wroteToTimeBuffer = true;
        }

    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    insts[tid].clear();

    // Clear the skid buffer in case it has any data in it.
    skidBuffer[tid].clear();

    doSquash(squash_seq_num, tid);
}

template <class Impl>
void
DefaultRename<Impl>::tick()
{
    wroteToTimeBuffer = false;

    blockThisCycle = false;

    bool status_change = false;

    toIEWWIndex = 0;
    toCommitIndex = 0;
    
    instsWakeNum = 0;//number of insts wake up these tick()
    
    sortInsts();

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check stall and squash signals.
    //std::cout<<"in rename:"<<std::endl;
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(Rename, "Processing [tid:%i]\n", tid);

        status_change = checkSignalsAndUpdate(tid) || status_change;
         
        renameWakeUpInsts(tid);
        
        rename(status_change, tid);
   
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    threads = activeThreads->begin();

    while (threads != end) {
        ThreadID tid = *threads++;

        // If we committed this cycle then doneSeqNum will be > 0
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            renameStatus[tid] != Squashing) {

            removeFromHistoryExt(fromCommit->commitInfo[tid].doneSeqNum,
                                  tid);
            removeFromHistorySec(fromCommit->commitInfo[tid].doneSeqNum,
                                  tid);
        }
    }

    // @todo: make into updateProgress function
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        //std::cout<<"rename tick,"<<"before toIQ:"<<toIQInProgress[tid]<<",toRob:"<<toRobInProgress[tid]<<",storesInProgress:"<<storesInProgress[tid]<<std::endl;
        //std::cout<<"from IEW:"<<"dispatched:"<<fromIEW->iewInfo[tid].dispatched;
        //std::cout<<" from commit:getfromrename:"<<fromCommit->commitInfo[tid].getFromRename<<std::endl;
        //std::cout<<",dispatchedToSQ:"<<fromIEW->iewInfo[tid].dispatchedToSQ;
        
        toIQInProgress[tid] -= fromIEW->iewInfo[tid].dispatched;
        toRobInProgress[tid] -= fromCommit->commitInfo[tid].getFromRename;
        loadsInProgress[tid] -= fromIEW->iewInfo[tid].dispatchedToLQ;
        storesInProgress[tid] -= fromIEW->iewInfo[tid].dispatchedToSQ;
        
        //std::cout<<"rename tick,"<<"after: toIQInProgress:"<<toIQInProgress[tid]<<",toRobInProgress"<<toRobInProgress[tid]<<"storesInProgress:"<<storesInProgress[tid]<<std::endl;

        assert(loadsInProgress[tid] >= 0);
        assert(storesInProgress[tid] >= 0);
        assert(toIQInProgress[tid] >=0);
        assert(toRobInProgress[tid] >=0);
        //std::cout<<endl;
    }

}

template<class Impl>
void
DefaultRename<Impl>::rename(bool &status_change, ThreadID tid)
{
    // If status is Running or idle,
    //     call renameInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from decode
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (renameStatus[tid] == Blocked) {
        ++renameBlockCycles;
    } else if (renameStatus[tid] == Squashing) {
        ++renameSquashCycles;
    } else if (renameStatus[tid] == SerializeStall) {
        ++renameSerializeStallCycles;
        // If we are currently in SerializeStall and resumeSerialize
        // was set, then that means that we are resuming serializing
        // this cycle.  Tell the previous stages to block.
        if (resumeSerialize) {
            resumeSerialize = false;
            block(tid);
            toDecode->renameUnblock[tid] = false;
        }
    } else if (renameStatus[tid] == Unblocking) {
        if (resumeUnblocking) {
            block(tid);
            resumeUnblocking = false;
            toDecode->renameUnblock[tid] = false;
        }
    }

    if (renameStatus[tid] == Running ||
        renameStatus[tid] == Idle) {
        DPRINTF(Rename, "[tid:%u]: Not blocked, so attempting to run "
                "stage.\n", tid);
        //renameWakeUpInsts(tid);

        renameInsts(tid);
    } else if (renameStatus[tid] == Unblocking) {
        renameInsts(tid);
        //renameWakeUpInsts(tid);

        if (validInsts()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        // If we switched over to blocking, then there's a potential for
        // an overall status change.
        status_change = unblock(tid) || status_change || blockThisCycle;
    }
}

template <class Impl>
void
DefaultRename<Impl>::renameInsts(ThreadID tid)
{
    // Instructions can be either in the skid buffer or the queue of
    // instructions coming from decode, depending on the status.
    int insts_available = renameStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();

    // Check the decode queue to see if instructions are available.
    // If there are no available instructions to rename, then do nothing.
    if (insts_available == 0) {
        DPRINTF(Rename, "[tid:%u]: Nothing to do, breaking out early.\n",
                tid);
        // Should I change status to idle?
        ++renameIdleCycles;
        return;
    } else if (renameStatus[tid] == Unblocking) {
        ++renameUnblockCycles;
    } else if (renameStatus[tid] == Running) {
        ++renameRunCycles;
    }
    
    if(getLTPStatus(tid) == true) ++openLTPCycles;
    else ++closeLTPCycles;

    // Will have to do a different calculation for the number of free
    // entries.
    int free_rob_entries = calcFreeROBEntries(tid);
    int free_iq_entries  = calcFreeIQEntries(tid);
    int min_free_entries = free_rob_entries;

    FullSource source = ROB;

    if (free_iq_entries < min_free_entries) {
        min_free_entries = free_iq_entries;
        source = IQ;
    }

    // Check if there's any space left.
    if (min_free_entries <= 0) {
        DPRINTF(Rename, "[tid:%u]: Blocking due to no free ROB/IQ/ "
                "entries.\n"
                "ROB has %i free entries.\n"
                "IQ has %i free entries.\n",
                tid,
                free_rob_entries,
                free_iq_entries);

        blockThisCycle = true;

        block(tid);

        incrFullStat(source);

        return;
    } else if (min_free_entries < insts_available) {
        DPRINTF(Rename, "[tid:%u]: Will have to block this cycle."
                "%i insts available, but only %i insts can be "
                "renamed due to ROB/IQ/LSQ limits.\n",
                tid, insts_available, min_free_entries);

        insts_available = min_free_entries;

        blockThisCycle = true;

        incrFullStat(source);
    }

    InstQueue &insts_to_rename = renameStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    DPRINTF(Rename, "[tid:%u]: %i available instructions to "
            "send iew.\n", tid, insts_available);

    DPRINTF(Rename, "[tid:%u]: %i insts pipelining from Rename | %i insts "
            "dispatched to IQ last cycle.\n",
            tid, toIQInProgress[tid], fromIEW->iewInfo[tid].dispatched);

    // Handle serializing the next instruction if necessary.
    if (serializeOnNextInst[tid]) {
        if (emptyROB[tid] && toRobInProgress[tid] == 0) {
            // ROB already empty; no need to serialize.
            serializeOnNextInst[tid] = false;
        } else if (!insts_to_rename.empty()) {
            insts_to_rename.front()->setSerializeBefore();
        }
    }

    int renamed_insts = 0;
    int to_rob_insts = 0;
    int park_count = 0;


    while (insts_available > 0 &&  toIEWWIndex < renameWidth && toCommitIndex < renameWidth) {
        DPRINTF(Rename, "[tid:%u]: Sending instructions to IEW.\n", tid);

        assert(!insts_to_rename.empty());

        DynInstPtr inst = insts_to_rename.front();

        //For all kind of instructions, check ROB and IQ first
        //For load instruction, check LQ size and take into account the inflight loads
        //For store instruction, check SQ size and take into account the inflight stores

        if (inst->isLoad()) {
            if (calcFreeLQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Cannot rename due to no free LQ\n");
                source = LQ;
                incrFullStat(source);
                break;
            }
        }

        if (inst->isStore() || inst->isAtomic()) {
            if (calcFreeSQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Cannot rename due to no free SQ\n");
                source = SQ;
                incrFullStat(source);
                break;
            }
        }

        insts_to_rename.pop_front();

        if (renameStatus[tid] == Unblocking) {
            DPRINTF(Rename,"[tid:%u]: Removing [sn:%lli] PC:%s from rename "
                    "skidBuffer\n", tid, inst->seqNum, inst->pcState());
        }

        if (inst->isSquashed()) {
            DPRINTF(Rename, "[tid:%u]: instruction %i with PC %s is "
                    "squashed, skipping.\n", tid, inst->seqNum,
                    inst->pcState());

            ++renameSquashedInsts;

            // Decrement how many instructions are available.
            --insts_available;

            continue;
        }

        DPRINTF(Rename, "[tid:%u]: renameInst,Processing instruction [sn:%lli] with "
                "PC %s.\n", tid, inst->seqNum, inst->pcState());

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (!renameMap[tid]->canRename(inst->numIntDestRegs(),
                                       inst->numFPDestRegs(),
                                       inst->numVecDestRegs(),
                                       inst->numVecElemDestRegs(),
                                       inst->numVecPredDestRegs(),
                                       inst->numCCDestRegs())) {
            DPRINTF(Rename, "Blocking due to lack of free "
                    "physical registers to rename to.\n");
            blockThisCycle = true;
            insts_to_rename.push_front(inst);
            ++renameFullRegistersEvents;

            break;
        }

        // Handle serializeAfter/serializeBefore instructions.
        // serializeAfter marks the next instruction as serializeBefore.
        // serializeBefore makes the instruction wait in rename until the ROB
        // is empty.

        // In this model, IPR accesses are serialize before
        // instructions, and store conditionals are serialize after
        // instructions.  This is mainly due to lack of support for
        // out-of-order operations of either of those classes of
        // instructions.
        if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
            !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize before instruction encountered.\n");

            if (!inst->isTempSerializeBefore()) {
                renamedSerializing++;
                inst->setSerializeHandled();
            } else {
                renamedTempSerializing++;
            }

            // Change status over to SerializeStall so that other stages know
            // what this is blocked on.
            renameStatus[tid] = SerializeStall;

            serializeInst[tid] = inst;

            blockThisCycle = true;

            break;
        } else if ((inst->isStoreConditional() || inst->isSerializeAfter()) &&
                   !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize after instruction encountered.\n");

            renamedSerializing++;

            inst->setSerializeHandled();

            serializeAfter(insts_to_rename, tid);
        }
        
        if(inst->isAtomic() || inst->isLoad() || inst->isStore()) {
            cpu->iew.ldstQueue.insertReadyToLSQ(inst);
        }     
           
        if(gateLTP[tid] == true && (inst->urgent == false || findSrcParkBit(inst) == true)){
             //write in to RAT :set pc and set park bit in Rat
            if(inst->urgent == false) {
                DPRINTF(Rename,"park into LTP because of inst is non-urgent [sn:%i]\n",inst->seqNum);
            } else {
                DPRINTF(Rename,"park into LTP because of source have parkBit [sn:%i]\n",inst->seqNum);
            }
            renameSrcBeforePark(inst);
            bool resu = renameDestBeforePark(inst);
            DPRINTF(Rename,"ResetDestBeforePark success:%d\n",resu);            
             //insert into LTP
            resu = insertLTP(inst,tid);
            DPRINTF(Rename,"insert in LTP success:%d\n",resu);            
             
             //set NoneedExe
            inst->fromLTP = true; 
            inst->noNeedExe = true;
            park_count ++; 
        }else{

            renameSrcRegs(inst, inst->threadNumber);
  
            renameDestRegs(inst, inst->threadNumber);
                    
            if (inst->isAtomic() || inst->isStore()) {
                storesInProgress[tid]++;
            } else if (inst->isLoad()) {
                loadsInProgress[tid]++;
            }

            ++renamed_insts;
            // Notify potential listeners that source and destination registers for
            // this instruction have been renamed.
            ppRename->notify(inst);
            inst->noNeedExe = false;
            inst->fromLTP = false;
            toIEWW->insts[toIEWWIndex] = inst;
            ++(toIEWW->size);
            // Increment which instruction we're on.
            ++toIEWWIndex;
            //std::cout<<"to iew,commit size:"<<toCommit->size<<" iew size:"<<toIEWW->size<<"noNeedExe:"<<inst->noNeedExe<<" ";
            //inst->dump();
        }
        toCommit->insts[toCommitIndex] = inst;
        ++(toCommit->size);

        // Increment which instruction we're on.
        ++toCommitIndex;

        // Decrement how many instructions are available.
        --insts_available;
        
        ++to_rob_insts;
        //std::cout<<"to commit,commit size:"<<toCommit->size<<" iew size:"<<toIEWW->size<<"noNeedExe:"<<inst->noNeedExe<<" ";
        //inst->dump();
       
         
    }
    numLTPInstsPer.sample(park_count);

    toIQInProgress[tid] += renamed_insts;
    toRobInProgress[tid] += to_rob_insts;
    renameRenamedInsts += renamed_insts;

    // If we wrote to the time buffer, record this.
    if (toIEWWIndex || toCommitIndex) {
        wroteToTimeBuffer = true;
    }

    // Check if there's any instructions left that haven't yet been renamed.
    // If so then block.
    if (insts_available) {
        blockThisCycle = true;
    }

    if (blockThisCycle) {
        block(tid);
        toDecode->renameUnblock[tid] = false;
    }
}

template<class Impl>
void
DefaultRename<Impl>::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop_front();

        assert(tid == inst->threadNumber);

        DPRINTF(Rename, "[tid:%u]: Inserting [sn:%lli] PC: %s into Rename "
                "skidBuffer\n", tid, inst->seqNum, inst->pcState());

        ++renameSkidInsts;

        skidBuffer[tid].push_back(inst);
    }

    if (skidBuffer[tid].size() > skidBufferMax)
    {
        typename InstQueue::iterator it;
        warn("Skidbuffer contents:\n");
        for (it = skidBuffer[tid].begin(); it != skidBuffer[tid].end(); it++)
        {
            warn("[tid:%u]: %s [sn:%i].\n", tid,
                    (*it)->staticInst->disassemble(inst->instAddr()),
                    (*it)->seqNum);
        }
        panic("Skidbuffer Exceeded Max Size");
    }
}

template <class Impl>
void
DefaultRename<Impl>::sortInsts()
{
    int insts_from_decode = fromDecode->size;
    for (int i = 0; i < insts_from_decode; ++i) {
        const DynInstPtr &inst = fromDecode->insts[i];
        insts[inst->threadNumber].push_back(inst);
#if TRACING_ON
        if (DTRACE(O3PipeView)) {
            inst->renameTick = curTick() - inst->fetchTick;
        }
#endif
    }
}

template<class Impl>
bool
DefaultRename<Impl>::skidsEmpty()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

template<class Impl>
void
DefaultRename<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (renameStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Rename will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(O3CPU::RenameIdx);
        }
    } else {
        // If it's not unblocking, then rename will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(O3CPU::RenameIdx);
        }
    }
}

template <class Impl>
bool
DefaultRename<Impl>::block(ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Blocking.\n", tid);

    // Add the current inputs onto the skid buffer, so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    // Only signal backwards to block if the previous stages do not think
    // rename is already blocked.
    if (renameStatus[tid] != Blocked) {
        // If resumeUnblocking is set, we unblocked during the squash,
        // but now we're have unblocking status. We need to tell earlier
        // stages to block.
        if (resumeUnblocking || renameStatus[tid] != Unblocking) {
            toDecode->renameBlock[tid] = true;
            toDecode->renameUnblock[tid] = false;
            wroteToTimeBuffer = true;
        }

        // Rename can not go from SerializeStall to Blocked, otherwise
        // it would not know to complete the serialize stall.
        if (renameStatus[tid] != SerializeStall) {
            // Set status to Blocked.
            renameStatus[tid] = Blocked;
            return true;
        }
    }

    return false;
}

template <class Impl>
bool
DefaultRename<Impl>::unblock(ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Trying to unblock.\n", tid);

    // Rename is done unblocking if the skid buffer is empty.
    if (skidBuffer[tid].empty() && renameStatus[tid] != SerializeStall) {

        DPRINTF(Rename, "[tid:%u]: Done unblocking.\n", tid);

        toDecode->renameUnblock[tid] = true;
        wroteToTimeBuffer = true;

        renameStatus[tid] = Running;
        return true;
    }

    return false;
}

template <class Impl>
void
DefaultRename<Impl>::doSquash(const InstSeqNum &squashed_seq_num, ThreadID tid)
{
    typename std::list<RenameHistoryExt>::iterator hb_it_ext =
        historyBufferExt[tid].begin();
    DPRINTF(Rename, "[tid:%u]: history bufferExt's begin "
        "seq number %i.\n", tid, hb_it_ext->instSeqNum);
    // After a syscall squashes everything, the history buffer may be empty
    // but the ROB may still be squashing instructions.
    // Go through the most recent instructions, undoing the mappings
    // they did and freeing up the registers.
    while (!historyBufferExt[tid].empty() &&
           hb_it_ext->instSeqNum > squashed_seq_num) {
        assert(hb_it_ext != historyBufferExt[tid].end());

        DPRINTF(Rename, "[tid:%u]: Removing history entry with sequence "
                "number [sn:%i].\n", tid, hb_it_ext->instSeqNum);

        // Undo the rename mapping only if it was really a change.
        // Special regs that are not really renamed (like misc regs
        // and the zero reg) can be recognized because the new mapping
        // is the same as the old one.  While it would be merely a
        // waste of time to update the rename table, we definitely
        // don't want to put these on the free list.
        if (hb_it_ext->newPhysReg != hb_it_ext->prevPhysReg) {
            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            //DPRINTF(Rename,"set entryext, Arch reg[%s]: %i,pc%s:,parkBit:%d\n",       
            //hb_it_ext->archReg.className(),hb_it_ext->archReg.index(),hb_it_ext->pc,hb_it_ext->parkBit);
            if(hb_it_ext->prevPhysReg == NULL) {
                renameMap[tid]->setEntryExtNULL(hb_it_ext->archReg, 
                    hb_it_ext->pc,hb_it_ext->parkBit);
            }else {
                renameMap[tid]->setEntryExt(hb_it_ext->archReg, 
                    hb_it_ext->prevPhysReg,hb_it_ext->pc,hb_it_ext->parkBit);
            }

            // Put the renamed physical register back on the free list.
            if(hb_it_ext->newPhysReg !=NULL) 
            {
                DPRINTF(Rename,"Freeing up new rename of reg %s(%s),[sn:%i]",
                    hb_it_ext->newPhysReg->index(),hb_it_ext->newPhysReg->className(),hb_it_ext->instSeqNum); 
                freeList->addReg(hb_it_ext->newPhysReg);
            }
        }

        // Notify potential listeners that the register mapping needs to be
        // removed because the instruction it was mapped to got squashed. Note
        // that this is done before hb_it is incremented.
        ppSquashInRename->notify(std::make_pair(hb_it_ext->instSeqNum,
                                                hb_it_ext->newPhysReg));

        historyBufferExt[tid].erase(hb_it_ext++);

        ++renameUndoneMaps;
    }

    typename std::list<RenameHistorySec>::iterator hb_it_sec =
        historyBufferSec[tid].begin();
    DPRINTF(Rename, "[tid:%u]: history buffer's begin "
        "seq number %i.\n", tid, hb_it_sec->instSeqNum);
    // After a syscall squashes everything, the history buffer may be empty
    // but the ROB may still be squashing instructions.
    // Go through the most recent instructions, undoing the mappings
    // they did and freeing up the registers.
    while (!historyBufferSec[tid].empty() &&
           hb_it_sec->instSeqNum > squashed_seq_num) {
        assert(hb_it_sec != historyBufferSec[tid].end());

        DPRINTF(Rename, "[tid:%u]: Removing history entry with sequence "
                "number %i.\n", tid, hb_it_sec->instSeqNum);

        // Undo the rename mapping only if it was really a change.
        // Special regs that are not really renamed (like misc regs
        // and the zero reg) can be recognized because the new mapping
        // is the same as the old one.  While it would be merely a
        // waste of time to update the rename table, we definitely
        // don't want to put these on the free list.
        if (hb_it_sec->newPhysReg != hb_it_sec->prevPhysReg) {
            if(hb_it_sec->prevPhysReg == NULL) {
                renameMap[tid]->setEntrySecNULL(hb_it_sec->archReg);
            }else {
                renameMap[tid]->setEntrySec(hb_it_sec->archReg, hb_it_sec->prevPhysReg);
            }
            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            
            // Put the renamed physical register back on the free list.
            assert(hb_it_sec->newPhysReg != NULL);
            DPRINTF(Rename,"Freeing up new rename of reg %s(%s),[sn:%i]",
                hb_it_sec->newPhysReg->index(),hb_it_sec->newPhysReg->className(),hb_it_sec->instSeqNum); 
            freeList->addReg(hb_it_sec->newPhysReg);
        }

        // Notify potential listeners that the register mapping needs to be
        // removed because the instruction it was mapped to got squashed. Note
        // that this is done before hb_it is incremented.
        ppSquashInRename->notify(std::make_pair(hb_it_sec->instSeqNum,
                                                hb_it_sec->newPhysReg));

        historyBufferSec[tid].erase(hb_it_sec++);

        ++renameUndoneMaps;
    }

    //squash LTP
    while(secRenameQueue[tid].empty()!= true) {
        if(secRenameQueue[tid].back()->seqNum > squashed_seq_num) {
            DPRINTF(Rename, "[tid:%u]: Removing inst seqNum:%i in secRenameQueue.\n"
                ,tid, secRenameQueue[tid].back()->seqNum);
            secRenameQueue[tid].pop_back();
        } else {
            break;
        }
    }
    DPRINTF(Rename,"[tid:%u]:secRenameQueue size is %d\n",tid,secRenameQueue[tid].size()); 
 
    while(LTP[tid].empty()!=true) {
        if(LTP[tid].back()->seqNum > squashed_seq_num) {
            DPRINTF(Rename, "[tid:%u]: Removing inst seqNum:%i in LTP.\n"
                ,tid, LTP[tid].back()->seqNum);
            LTP[tid].pop_back();
        } else {
            break;
        }
    }
    DPRINTF(Rename,"[tid:%u]:LTP size is %d\n",tid,LTP[tid].size()); 

    // Check if we need to change vector renaming mode after squashing
    cpu->switchRenameMode(tid, freeList);
}

template<class Impl>
void
DefaultRename<Impl>::removeFromHistoryExt(InstSeqNum inst_seq_num, ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Removing a committed instruction from the "
            "Ext history buffer %u (Extsize=%i), until [sn:%lli].\n",
            tid, tid, historyBufferExt[tid].size(),inst_seq_num);

    typename std::list<RenameHistoryExt>::iterator hb_it =
        historyBufferExt[tid].end();

    --hb_it;

    if (historyBufferExt[tid].empty()) {
        DPRINTF(Rename, "[tid:%u]: History buffer Ext is empty.\n", tid);
        return;
    } else if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "[tid:%u]: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n", tid);
        return;
    }

    // Commit all the renames up until (and including) the committed sequence
    // number. Some or even all of the committed instructions may not have
    // rename histories if they did not have destination registers that were
    // renamed.
    while (!historyBufferExt[tid].empty() &&
           hb_it != historyBufferExt[tid].end() &&
           hb_it->instSeqNum <= inst_seq_num) {


        // Don't free special phys regs like misc and zero regs, which
        // can be recognized because the new mapping is the same as
        // the old one.
        if (hb_it->newPhysReg != hb_it->prevPhysReg) {
            if(hb_it->prevPhysReg == NULL) {
                DPRINTF(Rename, "[tid:%u]: Freeing up older rename NULL,[sn:%lli].\n",
                tid, 
                hb_it->instSeqNum);
            }else{
                DPRINTF(Rename, "[tid:%u]: Freeing up older rename of reg %i (%s), "
                "[sn:%lli].\n",
                tid, hb_it->prevPhysReg->index(),
                hb_it->prevPhysReg->className(),
                hb_it->instSeqNum);

                freeList->addReg(hb_it->prevPhysReg);
            }
        }

        ++renameCommittedMaps;

        historyBufferExt[tid].erase(hb_it--);
    }
 
}

template<class Impl>
void
DefaultRename<Impl>::removeFromHistorySec(InstSeqNum inst_seq_num, ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Removing a committed instruction from the "
            "Sec history buffer %u (size=%i), until [sn:%lli].\n",
            tid, tid, historyBufferSec[tid].size(), inst_seq_num);

    //remove second rat
    typename std::list<RenameHistorySec>::iterator hb_it =
        historyBufferSec[tid].end();

    --hb_it;

    if (historyBufferSec[tid].empty()) {
        DPRINTF(Rename, "[tid:%u]: History buffer Sec is empty.\n", tid);
        return;
    } else if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "[tid:%u]: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n", tid);
        return;
    }

    // Commit all the renames up until (and including) the committed sequence
    // number. Some or even all of the committed instructions may not have
    // rename histories if they did not have destination registers that were
    // renamed.
    while (!historyBufferSec[tid].empty() &&
           hb_it != historyBufferSec[tid].end() &&
           hb_it->instSeqNum <= inst_seq_num) {

        // Don't free special phys regs like misc and zero regs, which
        // can be recognized because the new mapping is the same as
        // the old one.
        if (hb_it->newPhysReg != hb_it->prevPhysReg) {
            if(hb_it->prevPhysReg == NULL) {
                DPRINTF(Rename, "[tid:%u]: Freeing up older rename NULL,[sn:%lli].\n",
                tid, 
                hb_it->instSeqNum);
            }else{
                DPRINTF(Rename, "[tid:%u]: Freeing up older rename of reg %i (%s), "
                "[sn:%lli].\n",
                tid, hb_it->prevPhysReg->index(),
                hb_it->prevPhysReg->className(),
                hb_it->instSeqNum);

                freeList->addReg(hb_it->prevPhysReg);
            }
        }

        ++renameCommittedMaps;

        historyBufferSec[tid].erase(hb_it--);
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameSrcRegs(const DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // operands, and redirect them to the right physical register.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
        const RegId& src_reg = inst->srcRegIdx(src_idx);
        PhysRegIdPtr renamed_reg;
        
        if(true == map->lookupParkBit(tc->flattenRegId(src_reg))) {
            renamed_reg = map->lookupSec(tc->flattenRegId(src_reg));
        }else{
            renamed_reg = map->lookupExt(tc->flattenRegId(src_reg));
        }

        switch (src_reg.classValue()) {
          case IntRegClass:
            intRenameLookups++;
            break;
          case FloatRegClass:
            fpRenameLookups++;
            break;
          case VecRegClass:
          case VecElemClass:
            vecRenameLookups++;
            break;
          case VecPredRegClass:
            vecPredRenameLookups++;
            break;
          case CCRegClass:
          case MiscRegClass:
            break;

          default:
            panic("Invalid register class: %d.", src_reg.classValue());
        }

        DPRINTF(Rename, "[tid:%u]: Looking up %s arch reg %i"
                ", got phys reg %i (%s)\n", tid,
                src_reg.className(), src_reg.index(),
                renamed_reg->index(),
                renamed_reg->className());

        inst->renameSrcReg(src_idx, renamed_reg);
        
        //if the current inst is urgent then it source inst is urgent and need add to UIT
        if(inst->urgent == true){
            TheISA::PCState temp = map->getSourceInstPC(tc->flattenRegId(src_reg));
            if(temp != 0) {
                decode_ptr->urgInsert(inst, tid);
                DPRINTF(Rename, "set Source inst into UIT");
                inst->urgent = true;
            }
        }
    
        // See if the register is ready or not.
        if (scoreboard->getReg(renamed_reg)) {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());

            inst->markSrcRegReady(src_idx);
            /*
            uint64_t temp = 0;
            if(renamed_reg->isIntPhysReg() == true) {
                cpu->readIntReg(renamed_reg);
                std::cout<<"SN"<<inst->seqNum<<"get source op["<<src_idx<<"] value is:"<<temp<<std::endl;
            }*/
        } else {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is not ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());
        }

        ++renameRenameLookups;
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameDestRegs(const DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_dest_regs = inst->numDestRegs();

    // Rename the destination registers.
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
        const RegId& dest_reg = inst->destRegIdx(dest_idx);
        typename RenameMap::RenameInfoExt rename_result;

        RegId flat_dest_regid = tc->flattenRegId(dest_reg);

        rename_result = map->rename(flat_dest_regid,inst->pcState());

        inst->flattenDestReg(dest_idx, flat_dest_regid);

        // Mark Scoreboard entry as not ready
        scoreboard->unsetReg(rename_result.newPhysReg);

        DPRINTF(Rename, "[tid:%u]: Renaming arch reg %i (%s) to physical "
                "reg %i (%i).\n", tid, dest_reg.index(),
                dest_reg.className(),
                rename_result.newPhysReg->index(),
                rename_result.newPhysReg->flatIndex());

        // Record the rename information so that a history can be kept.
        RenameHistoryExt hb_entry(inst->seqNum, flat_dest_regid,
                               rename_result.newPhysReg,
                               rename_result.prevPhysReg,
                               rename_result.prevpc,
                               rename_result.prevparkBit);

        historyBufferExt[tid].push_front(hb_entry);

        DPRINTF(Rename, "[tid:%u]: Adding instruction to history buffer "
                "(size=%i), [sn:%lli].\n",tid,
                historyBufferExt[tid].size(),
                (*historyBufferExt[tid].begin()).instSeqNum);

        // Tell the instruction to rename the appropriate destination
        // register (dest_idx) to the new physical register
        // (rename_result.first), and record the previous physical
        // register that the same logical register was renamed to
        // (rename_result.second).
        inst->renameDestReg(dest_idx,
                            rename_result.newPhysReg,
                            rename_result.prevPhysReg);

        ++renameRenamedOperands;
    }
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeROBEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].robEntries -
                  (toRobInProgress[tid] - fromCommit->commitInfo[tid].getFromRename);

    //DPRINTF(Rename,"[tid:%i]: %i rob free\n",tid,num_free);

    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeIQEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].iqEntries -
                  (toIQInProgress[tid] - fromIEW->iewInfo[tid].dispatched);

    //DPRINTF(Rename,"[tid:%i]: %i iq free\n",tid,num_free);

    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeLQEntries(ThreadID tid)
{
        int num_free = freeEntries[tid].lqEntries -
                                  (loadsInProgress[tid] - fromIEW->iewInfo[tid].dispatchedToLQ);
        DPRINTF(Rename, "calcFreeLQEntries: free lqEntries: %d, loadsInProgress: %d, "
                "loads dispatchedToLQ: %d\n", freeEntries[tid].lqEntries,
                loadsInProgress[tid], fromIEW->iewInfo[tid].dispatchedToLQ);
        return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeSQEntries(ThreadID tid)
{
        int num_free = freeEntries[tid].sqEntries -
                                  (storesInProgress[tid] - fromIEW->iewInfo[tid].dispatchedToSQ);
        DPRINTF(Rename, "calcFreeSQEntries: free sqEntries: %d, storesInProgress: %d, "
                "stores dispatchedToSQ: %d\n", freeEntries[tid].sqEntries,
                storesInProgress[tid], fromIEW->iewInfo[tid].dispatchedToSQ);
        return num_free;
}

template <class Impl>
unsigned
DefaultRename<Impl>::validInsts()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromDecode->size; i++) {
        if (!fromDecode->insts[i]->isSquashed())
            inst_count++;
    }

    return inst_count;
}

template <class Impl>
void
DefaultRename<Impl>::readStallSignals(ThreadID tid)
{
    if (fromIEW->iewBlock[tid]) {
        stalls[tid].iew = true;
    }

    if (fromIEW->iewUnblock[tid]) {
        assert(stalls[tid].iew);
        stalls[tid].iew = false;
    }
}

template <class Impl>
bool
DefaultRename<Impl>::checkStall(ThreadID tid)
{
    bool ret_val = false;

    if (stalls[tid].iew) {
        DPRINTF(Rename,"[tid:%i]: Stall from IEW stage detected.\n", tid);
        ret_val = true;
    } else if (calcFreeROBEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: ROB has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeIQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: IQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeLQEntries(tid) <= 0 && calcFreeSQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: LSQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameMap[tid]->numFreeEntries() <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: RenameMap has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameStatus[tid] == SerializeStall &&
               (!emptyROB[tid] || toRobInProgress[tid])) {
        DPRINTF(Rename,"[tid:%i]: Stall: Serialize stall and ROB is not "
                "empty.\n",
                tid);
        ret_val = true;
    }

    return ret_val;
}

template <class Impl>
void
DefaultRename<Impl>::readFreeEntries(ThreadID tid)
{
    if (fromIEW->iewInfo[tid].usedIQ)
        freeEntries[tid].iqEntries = fromIEW->iewInfo[tid].freeIQEntries;

    if (fromIEW->iewInfo[tid].usedLSQ) {
        freeEntries[tid].lqEntries = fromIEW->iewInfo[tid].freeLQEntries;
        freeEntries[tid].sqEntries = fromIEW->iewInfo[tid].freeSQEntries;
    }

    if (fromCommit->commitInfo[tid].usedROB) {
        freeEntries[tid].robEntries =
            fromCommit->commitInfo[tid].freeROBEntries;
        emptyROB[tid] = fromCommit->commitInfo[tid].emptyROB;
    }

    DPRINTF(Rename, "[tid:%i]: Free IQ: %i, Free ROB: %i, "
                    "Free LQ: %i, Free SQ: %i, FreeRM %i(%i %i %i %i %i)\n",
            tid,
            freeEntries[tid].iqEntries,
            freeEntries[tid].robEntries,
            freeEntries[tid].lqEntries,
            freeEntries[tid].sqEntries,
            renameMap[tid]->numFreeEntries(),
            renameMap[tid]->numFreeIntEntries(),
            renameMap[tid]->numFreeFloatEntries(),
            renameMap[tid]->numFreeVecEntries(),
            renameMap[tid]->numFreePredEntries(),
            renameMap[tid]->numFreeCCEntries());

    DPRINTF(Rename, "[tid:%i]: %i instructions not yet in ROB\n",
            tid, toRobInProgress[tid]);

}

template <class Impl>
bool
DefaultRename<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if necessary.
    // If status was blocked
    //     check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.
    // If status was serialize stall
    //     check if ROB is empty and no insts are in flight to the ROB

    readFreeEntries(tid);
    readStallSignals(tid);

    if (fromCommit->commitInfo[tid].squash) {
        DPRINTF(Rename, "[tid:%u]: Squashing instructions due to squash from "
                "commit.\n", tid);

        squash(fromCommit->commitInfo[tid].doneSeqNum, tid);

        return true;
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (renameStatus[tid] == Blocked) {
        DPRINTF(Rename, "[tid:%u]: Done blocking, switching to unblocking.\n",
                tid);

        renameStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (renameStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        if (resumeSerialize) {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to serialize.\n",
                    tid);

            renameStatus[tid] = SerializeStall;
            return true;
        } else if (resumeUnblocking) {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to unblocking.\n",
                    tid);
            renameStatus[tid] = Unblocking;
            return true;
        } else {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to running.\n",
                    tid);

            renameStatus[tid] = Running;
            return false;
        }
    }

    if (renameStatus[tid] == SerializeStall) {
        // Stall ends once the ROB is free.
        DPRINTF(Rename, "[tid:%u]: Done with serialize stall, switching to "
                "unblocking.\n", tid);

        DynInstPtr serial_inst = serializeInst[tid];

        renameStatus[tid] = Unblocking;

        unblock(tid);

        DPRINTF(Rename, "[tid:%u]: in check signal,Processing instruction [%lli] with "
                "PC %s.\n", tid, serial_inst->seqNum, serial_inst->pcState());

        // Put instruction into queue here.
        serial_inst->clearSerializeBefore();

        if (!skidBuffer[tid].empty()) {
            skidBuffer[tid].push_front(serial_inst);
        } else {
            insts[tid].push_front(serial_inst);
        }

        DPRINTF(Rename, "[tid:%u]: Instruction must be processed by rename."
                " Adding to front of list.\n", tid);

        serializeInst[tid] = NULL;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause rename to change its status.  Rename remains the same as before.
    return false;
}

template<class Impl>
void
DefaultRename<Impl>::serializeAfter(InstQueue &inst_list, ThreadID tid)
{
    if (inst_list.empty()) {
        // Mark a bit to say that I must serialize on the next instruction.
        serializeOnNextInst[tid] = true;
        return;
    }

    // Set the next instruction as serializing.
    inst_list.front()->setSerializeBefore();
}

template <class Impl>
inline void
DefaultRename<Impl>::incrFullStat(const FullSource &source)
{
    switch (source) {
      case ROB:
        ++renameROBFullEvents;
        break;
      case IQ:
        ++renameIQFullEvents;
        break;
      case LQ:
        ++renameLQFullEvents;
        break;
      case SQ:
        ++renameSQFullEvents;
        break;
      default:
        panic("Rename full stall stat should be incremented for a reason!");
        break;
    }
}

template <class Impl>
void
DefaultRename<Impl>::dumpHistory()
{
    typename std::list<RenameHistorySec>::iterator buf_itt;

    for (ThreadID tid = 0; tid < numThreads; tid++) {

        buf_itt = historyBufferSec[tid].begin();

        while (buf_itt != historyBufferSec[tid].end()) {
            cprintf("Seq num: %i\nArch reg[%s]: %i New phys reg:"
                    " %i[%s] Old phys reg: %i[%s]\n",
                    (*buf_itt).instSeqNum,
                    (*buf_itt).archReg.className(),
                    (*buf_itt).archReg.index(),
                    (*buf_itt).newPhysReg->index(),
                    (*buf_itt).newPhysReg->className(),
                    (*buf_itt).prevPhysReg->index(),
                    (*buf_itt).prevPhysReg->className());

            buf_itt++;
        }
    }
    cprintf("\n");


    typename std::list<RenameHistoryExt>::iterator buf_it;

    for (ThreadID tid = 0; tid < numThreads; tid++) {

        buf_it = historyBufferExt[tid].begin();

        while (buf_it != historyBufferExt[tid].end()) {
            cprintf("Seq num: %i\nArch reg[%s]: %i New phys reg:"
                    " %i[%s] Old phys reg: %i[%s]\n",
                    (*buf_it).instSeqNum,
                    (*buf_it).archReg.className(),
                    (*buf_it).archReg.index(),
                    (*buf_it).newPhysReg != NULL?(*buf_it).newPhysReg->index():-1,
                    (*buf_it).newPhysReg != NULL?(*buf_it).newPhysReg->className():"-1",
                    (*buf_it).prevPhysReg != NULL?(*buf_it).prevPhysReg->index():-1,
                    (*buf_it).prevPhysReg != NULL?(*buf_it).prevPhysReg->className():"-1");

            buf_it++;
        }
    }
    cprintf("\n");

}


template <class Impl>
void
DefaultRename<Impl>::openLTP(ThreadID tid)
{
    if(gateLTP[tid] == true) return;
    else gateLTP[tid] = true;
    DPRINTF(Rename,"[tid:%u]:open LTP",tid);
}


//add for turn off LTP
template <class Impl>
void
DefaultRename<Impl>::closeLTP(ThreadID tid)
{
    if(gateLTP[tid] == false) return;
    else{
        gateLTP[tid] = false;
        while(LTP[tid].empty()!= true){
            wakeUpInst(LTP[tid].front());
        }
        DPRINTF(Rename,"[tid:%u]:close LTP wake up insts in LTP",tid);
        
    }
    DPRINTF(Rename,"[tid:%u]:close LTP",tid);
}

//get LTP status
template <class Impl>
bool
DefaultRename<Impl>::getLTPStatus(ThreadID tid)
{
    return gateLTP[tid];
}


//insert instruction to LTP 
template <class Impl>
bool
DefaultRename<Impl>::insertLTP(DynInstPtr &inst,ThreadID tid)
{
    if(LTP[tid].size() >= LTPMax) {
        DPRINTF(Rename, "the LTP is full wake up inst.\n");
        wakeUpInst(LTP[tid].front());
    }
    LTP[tid].push_back(inst);
    ++InLTPInsts;
    DPRINTF(Rename, "insert LTP LTP size:%d,secRenameQueue size:%d,sn:%i,LTP top is sn:%i.\n",
        LTP[tid].size(),secRenameQueue[tid].size(),inst->seqNum,LTP[tid].front()->seqNum);
    return true;
}

template <class Impl>
bool
DefaultRename<Impl>::wakeUpInst(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;
    if(LTP[tid].empty() == true) {
        DPRINTF(Rename, "the LTP is empty cannot wakeup,sn:%i.\n",inst->seqNum);
     
        return false;
    }
    DynInstPtr inst_top = LTP[tid].front();  
    
    if(inst->seqNum != inst_top->seqNum) {
        DPRINTF(Rename, "the LTP top is not this seq [sn:%i].the top is sn:%i,LTP size:%d,secRenameQueue size:%d. \n",
            inst->seqNum,inst_top->seqNum,LTP[tid].size(),secRenameQueue[tid].size());
        return false;
    }
    
    LTP[tid].pop_front();
    secRenameQueue[tid].push_back(inst_top);
    DPRINTF(Rename, "[tid:%u]:waiting in waked up queue sn:%i,LTP size:%d,secRenameQueue size:%d:\n"
           ,tid,inst->seqNum,LTP[tid].size(),secRenameQueue[tid].size());
    return true;
   
}

template <class Impl>
void
DefaultRename<Impl>::renameWakeUpInsts(ThreadID tid)
{
    // rename instruction coming from LTP
    int insts_avail = (int)secRenameQueue[tid].size();

    // Check the decode queue to see if instructions are available.
    // If there are no available instructions to rename, then do nothing.
    if (insts_avail == 0) {
        DPRINTF(Rename, "[tid:%u]: Rename wake up insts, inst_avail is 0, "
            "breaking out early.\n",tid);
        return;
    }
    DPRINTF(Rename, "[tid:%u]: Rename wake up insts, inst_avail is %d, "
            ".\n",tid,insts_avail);

    // Will have to do a different calculation for the number of free
    // entries.
    //int free_rob_entries = calcFreeROBEntries(tid);
    int free_iq_entries  = calcFreeIQEntries(tid);
    //int min_free_entries = free_rob_entries;
    int min_free_entries = free_iq_entries;

    //FullSource source = ROB;

    if (free_iq_entries < min_free_entries) {
        min_free_entries = free_iq_entries;
        //source = IQ;
    }

    // Check if there's any space left.
    if (min_free_entries <= 0) {
        DPRINTF(Rename, "[tid:%u]:  Rename wake up insts, Blocking due to no free IQ/ "
                "entries.\n"
                "IQ has %i free entries.\n",
                tid,
                free_iq_entries);
        return;
    } else if (min_free_entries < insts_avail) {
        DPRINTF(Rename, "[tid:%u]: Will have to block this cycle."
                "%i insts available, but only %i insts can be "
                "renamed due to ROB/IQ/LSQ limits.\n",
                tid, insts_avail, min_free_entries);

        insts_avail = min_free_entries;

    }

    DPRINTF(Rename, "[tid:%u]: %i available wake up instructions to "
        "send iew.\n", tid, insts_avail);


    int renamed_insts = 0;

    while (insts_avail > 0 &&  instsWakeNum < maxSecRename) {
        DPRINTF(Rename, "[tid:%u]: Wake up insts. Sending instructions to IEW.\n", tid);

        assert(!secRenameQueue[tid].empty());

        DynInstPtr inst = secRenameQueue[tid].front();

        //For all kind of instructions, check ROB and IQ first
        //For load instruction, check LQ size and take into account the inflight loads
        //For store iestruction, check SQ size and take into account the inflight stores

        if (inst->isLoad()) {
            if (calcFreeLQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Wake up insts.Cannot rename due to no free LQ\n");
                //source = LQ;
                //incrFullStat(source);
                break;
            }
        }

        if (inst->isStore() || inst->isAtomic()) {
            if (calcFreeSQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Wake up insts.Cannot rename due to no free SQ\n");
                //source = SQ;
                //incrFullStat(source);
                break;
            }
        }


        DPRINTF(Rename,"[tid:%u]: Removing [sn:%lli] PC:%s from wake up queue\n"
                    "", tid, inst->seqNum, inst->pcState());

        if (inst->isSquashed()) {
            DPRINTF(Rename, "[tid:%u]: instruction %i with PC %s is "
                    "squashed, skipping.when rename wake up instructions\n", 
                    tid, inst->seqNum,inst->pcState());

            secRenameQueue[tid].pop_front();
            ++renameSquashedInsts;

            // Decrement how many instructions are available.
            --insts_avail;

            continue;
        }

        DPRINTF(Rename, "[tid:%u]: in renname wake up inst,Processing instruction [sn:%lli] with "
                "PC %s.\n", tid, inst->seqNum, inst->pcState());

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (!renameMap[tid]->canRename(inst->numIntDestRegs(),
                                       inst->numFPDestRegs(),
                                       inst->numVecDestRegs(),
                                       inst->numVecElemDestRegs(),
                                       inst->numVecPredDestRegs(),
                                       inst->numCCDestRegs())) {
            DPRINTF(Rename, "Blocking due to lack of free "
                    "physical registers to rename to.\n");

            break;
        }
        secRenameQueue[tid].pop_front();
         
        // Handle serializeAfter/serializeBefore instructions.
        // serializeAfter marks the next instruction as serializeBefore.
        // serializeBefore makes the instruction wait in rename until the ROB
        // is empty.

        // In this model, IPR accesses are serialize before
        // instructions, and store conditionals are serialize after
        // instructions.  This is mainly due to lack of support for
        // out-of-order operations of either of those classes of
        // instructions.
        /*if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
            !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize before instruction encountered.\n");

            if (!inst->isTempSerializeBefore()) {
                renamedSerializing++;
                inst->setSerializeHandled();
            } else {
                renamedTempSerializing++;
            }*/

            // Change status over to SerializeStall so that other stages know
            // what this is blocked on.
          /*  renameStatus[tid] = SerializeStall;

            serializeInst[tid] = inst;

            blockThisCycle = true;

            break;
        } else if ((inst->isStoreConditional() || inst->isSerializeAfter()) &&
                   !inst->isSerializeHandled()) {
            DPRINTF(Rename, "Serialize after instruction encountered.\n");

            renamedSerializing++;

            inst->setSerializeHandled();

            serializeAfter(insts_to_rename, tid);
        }
        */
        DPRINTF(Rename, "Start to rename inst.\n");
        //std::cout<<"i am in wake up renamer inst";
        //inst->dump();
        //DPRINTF(Rename, "finish inst dump.\n");
        renameSrcRegsSec(inst, inst->threadNumber);

        renameDestRegsSec(inst, inst->threadNumber);

        if (inst->isAtomic() || inst->isStore()) {
            storesInProgress[tid]++;
        } else if (inst->isLoad()) {
            loadsInProgress[tid]++;
        }

        ++renamed_insts;
        // Notify potential listeners that source and destination registers for
        // this instruction have been renamed.
        ppRename->notify(inst);
       
        inst->noNeedExe = false;
        //inst->fromLTP = true;
        // Put instruction in rename queue.
        toIEWW->insts[toIEWWIndex] = inst;
        ++(toIEWW->size);

        // Increment which instruction we're on.
        ++toIEWWIndex;
       
        //the number of wake up and rename instruction 
        ++instsWakeNum;

        // Decrement how many instructions are available.
        --insts_avail;
        //std::cout<<inst->noNeedExe<<" ";
        //inst->dump();
    }

    toIQInProgress[tid] += renamed_insts;
    renameRenamedInsts += renamed_insts;

    // If we wrote to the time buffer, record this.
    if (toIEWWIndex) {
        wroteToTimeBuffer = true;
    }

}

template <class Impl>
inline void
DefaultRename<Impl>::renameDestRegsSec(const DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_dest_regs = inst->numDestRegs();

    // Rename the destination registers.
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
        const RegId& dest_reg = inst->destRegIdx(dest_idx);
        typename RenameMap::RenameInfoSec rename_result;
        RegId flat_dest_regid = tc->flattenRegId(dest_reg);

        rename_result = map->renameWakeUp(flat_dest_regid);

        //inst->flattenDestReg(dest_idx, flat_dest_regid);

        // Mark Scoreboard entry as not ready
        scoreboard->unsetReg(rename_result.first);

        DPRINTF(Rename, "[tid:%u]: Rename wake up.Renaming arch reg %i (%s) to physical "
                "reg %i (%i).\n", tid, dest_reg.index(),
                dest_reg.className(),
                rename_result.first->index(),
                rename_result.first->flatIndex());

        // Record the rename information so that a history can be kept.
        RenameHistorySec hb_entry(inst->seqNum, flat_dest_regid,
                               rename_result.first,
                               rename_result.second);
        /*
        if(historyBuffer[tid].empty() == true || historyBuffer[tid].front().instSeqNum < inst->seqNum) {
            historyBuffer[tid].push_front(hb_entry);
        } else if(historyBuffer[tid].back().instSeqNum > inst->seqNum){
            historyBuffer[tid].push_back(hb_entry);
        }else {
            HisIt iter=historyBuffer[tid].begin();
            for(iter=historyBuffer[tid].begin();iter!=historyBuffer[tid].end();iter ++) {
                if(iter->instSeqNum <= inst->seqNum) break;
            }
            historyBuffer[tid].insert(iter,hb_entry);
        }
        */
     
        historyBufferSec[tid].push_front(hb_entry);

        DPRINTF(Rename, "[tid:%u]: Adding instruction to history buffer Sec"
                "(size=%i), [sn:%lli].\n",tid,
                historyBufferSec[tid].size(),
                (*historyBufferSec[tid].begin()).instSeqNum);

        // Tell the instruction to rename the appropriate destination
        // register (dest_idx) to the new physical register
        // (rename_result.first), and record the previous physical
        // register that the same logical register was renamed to
        // (rename_result.second).
        inst->renameDestReg(dest_idx,
                            rename_result.first,
                            rename_result.second);

        ++renameRenamedOperands;
    }
    //dumpHistory();
}

template <class Impl>
bool
DefaultRename<Impl>::findSrcParkBit(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // operands, and redirect them to the right physical register.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
        const RegId& src_reg = inst->srcRegIdx(src_idx);

        if(true == map->lookupParkBit(tc->flattenRegId(src_reg))) {
            return true;
            break;
        }
    }
    return false;
}

template <class Impl>
bool
DefaultRename<Impl>::renameDestBeforePark(DynInstPtr &inst)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[inst->threadNumber];
    unsigned num_dest_regs = inst->numDestRegs();
    ThreadID tid = inst->threadNumber; 
    // Rename the destination registers.
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
        const RegId& dest_reg = inst->destRegIdx(dest_idx);
        typename RenameMap::RenameInfoExt rename_result;

        RegId flat_dest_regid = tc->flattenRegId(dest_reg);

        rename_result = map->renameBeforePark(flat_dest_regid,inst->pcState());
        
        inst->flattenDestReg(dest_idx, flat_dest_regid); 
        DPRINTF(Rename, "[tid:%u]: Rename before parking.Renaming arch reg %i (%s).\n", 
            inst->threadNumber, dest_reg.index(),dest_reg.className());

        // Record the rename information so that a history can be kept.
        RenameHistoryExt hb_entry(inst->seqNum, flat_dest_regid,
                               rename_result.newPhysReg,
                               rename_result.prevPhysReg,
                               rename_result.prevpc,
                               rename_result.prevparkBit);

        historyBufferExt[tid].push_front(hb_entry);

        DPRINTF(Rename, "[tid:%u]: Adding instruction to history buffer ext"
                "(size=%i), [sn:%lli].\n",inst->threadNumber,
                historyBufferExt[inst->threadNumber].size(),
                (*historyBufferExt[tid].begin()).instSeqNum);

        // Tell the instruction to rename the appropriate destination
        // register (dest_idx) to the new physical register
        // (rename_result.first), and record the previous physical
        // register that the same logical register was renamed to
        // (rename_result.second).
        inst->renameDestReg(dest_idx,
                            rename_result.newPhysReg,
                            rename_result.prevPhysReg);

        ++renameRenamedOperands;
    }   
    return true;
}

template <class Impl>
bool
DefaultRename<Impl>::setParkInRAT(DynInstPtr &inst)
{
    ThreadContext *tc = inst->tcBase();
    ThreadID tid = inst->threadNumber;
    RenameMap *map = renameMap[tid];
    unsigned num_dest_regs = inst->numDestRegs();
    bool result_set = true;
    // Set pc ans Park BIT
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
        const RegId& dest_reg = inst->destRegIdx(dest_idx);

        RegId flat_dest_regid = tc->flattenRegId(dest_reg);

        result_set = map->setParkBit(flat_dest_regid);
    }
    return result_set;
}

template <class Impl>
bool
DefaultRename<Impl>::renameSrcBeforePark(DynInstPtr &inst) 
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[inst->threadNumber];
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // operands, and redirect them to the right physical register.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
        const RegId& src_reg = inst->srcRegIdx(src_idx);
        PhysRegIdPtr renamed_reg;
        if(true == map->lookupParkBit(tc->flattenRegId(src_reg))) {
            inst->renameSrcReg(src_idx, NULL);
            continue;
        }
        renamed_reg = map->lookupExt(tc->flattenRegId(src_reg));
        switch (src_reg.classValue()) {
          case IntRegClass:
            intRenameLookups++;
            break;
          case FloatRegClass:
            fpRenameLookups++;
            break;
          case VecRegClass:
          case VecElemClass:
            vecRenameLookups++;
            break;
          case VecPredRegClass:
            vecPredRenameLookups++;
            break;
          case CCRegClass:
          case MiscRegClass:
            break;

          default:
            panic("Invalid register class: %d.", src_reg.classValue());
        }

        DPRINTF(Rename, "[tid:%u]: Looking up src index:%d,%s arch reg %i"
                ", got phys reg %i (%s)\n", inst->threadNumber,src_idx,
                src_reg.className(), src_reg.index(),
                renamed_reg->index(),
                renamed_reg->className());

        inst->renameSrcReg(src_idx, renamed_reg);
        
        //if the current inst is urgent then it source inst is urgent and need add to UIT
        if(inst->urgent == true){
            TheISA::PCState temp = map->getSourceInstPC(tc->flattenRegId(src_reg));
            if(temp != 0) {
                decode_ptr->urgInsert(inst, inst->threadNumber);
                DPRINTF(Rename, "insert Source inst into UIT");
            }
        }
    
        // See if the register is ready or not.
        if (scoreboard->getReg(renamed_reg)) {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is ready.\n", inst->threadNumber, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());

            inst->markSrcRegReady(src_idx);
            /*
            uint64_t temp = 0;
            if(renamed_reg->isIntPhysReg() == true) {
                cpu->readIntReg(renamed_reg);
                std::cout<<"SN"<<inst->seqNum<<"get source op["<<src_idx<<"] value is:"<<temp<<std::endl;
            }*/
        } else {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is not ready.\n", inst->threadNumber, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());
        }

        ++renameRenameLookups;
    }
    return true;
}

template <class Impl>
inline void
DefaultRename<Impl>::renameSrcRegsSec(const DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // operands, and redirect them to the right physical register.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
        const RegId& src_reg = inst->srcRegIdx(src_idx);
        PhysRegIdPtr renamed_reg;
        renamed_reg = inst->renamedSrcRegIdx(src_idx);
        if(renamed_reg != NULL){
            DPRINTF(Rename,"[tid:%u]: no need wake up rename, src index:%d,check success ,"
                "looking up %s arch reg %i, got phys reg %i (%s).\n",
                tid,src_idx,src_reg.className(),src_reg.index(),
                renamed_reg->index(),renamed_reg->className());
            continue;
        }

        renamed_reg = map->lookupSec(tc->flattenRegId(src_reg));
        switch (src_reg.classValue()) {
          case IntRegClass:
            intRenameLookups++;
            break;
          case FloatRegClass:
            fpRenameLookups++;
            break;
          case VecRegClass:
          case VecElemClass:
            vecRenameLookups++;
            break;
          case VecPredRegClass:
            vecPredRenameLookups++;
            break;
          case CCRegClass:
          case MiscRegClass:
            break;

          default:
            panic("Invalid register class: %d.", src_reg.classValue());
        }

        DPRINTF(Rename, "[tid:%u]: Looking up %s arch reg %i"
                ", got phys reg %i (%s)\n", tid,
                src_reg.className(), src_reg.index(),
                renamed_reg->index(),
                renamed_reg->className());

        inst->renameSrcReg(src_idx, renamed_reg);
        
        if (scoreboard->getReg(renamed_reg)) {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());

            inst->markSrcRegReady(src_idx);
            /*
            uint64_t temp = 0;
            if(renamed_reg->isIntPhysReg() == true) {
                cpu->readIntReg(renamed_reg);
                std::cout<<"SN"<<inst->seqNum<<"get source op["<<src_idx<<"] value is:"<<temp<<std::endl;
            }*/
        } else {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is not ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());
        }

        ++renameRenameLookups;
    }
}
#endif//__CPU_O3_RENAME_IMPL_HH__

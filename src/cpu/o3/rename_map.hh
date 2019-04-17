/*
 * Copyright (c) 2015-2017 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Steve Reinhardt
 */

#ifndef __CPU_O3_RENAME_MAP_HH__
#define __CPU_O3_RENAME_MAP_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "arch/types.hh"
#include "config/the_isa.hh"
#include "cpu/o3/free_list.hh"
#include "cpu/o3/regfile.hh"
#include "cpu/reg_class.hh"
#include "enums/VecRegRenameMode.hh"


/** to set up a second RAT for LTP instruction to rename */
struct ExtRAT {
    PhysRegIdPtr preg;
    TheISA::PCState pc;
    bool parkBit;
};

/**
 * Register rename map for a single class of registers (e.g., integer
 * or floating point).  Because the register class is implicitly
 * determined by the rename map instance being accessed, all
 * architectural register index parameters and values in this class
 * are relative (e.g., %fp2 is just index 2).
 */
class SimpleRenameMap
{
  private:
    using Arch2PhysMap = std::vector<PhysRegIdPtr>;
    /** The acutal arch-to-phys register map */
    Arch2PhysMap secmap;
    
    /** second map used to rename ltp instruction*/                
    using ExtArch2PhysMap = std::vector<ExtRAT>;
    ExtArch2PhysMap extmap;

  public:
    using sec_iterator = Arch2PhysMap::iterator;
    using sec_const_iterator = Arch2PhysMap::const_iterator;
    /** second RAT iterator*/
    using ext_iterator = ExtArch2PhysMap::iterator;
    using ext_const_iterator = ExtArch2PhysMap::const_iterator;
  private:

    /**
     * Pointer to the free list from which new physical registers
     * should be allocated in rename()
     */
    SimpleFreeList *freeList;

    /**
     * The architectural index of the zero register. This register is
     * mapped but read-only, so we ignore attempts to rename it via
     * the rename() method.  If there is no such register for this map
     * table, it should be set to an invalid index so that it never
     * matches.
     */
    RegId zeroReg;

  public:

    SimpleRenameMap();

    ~SimpleRenameMap() {};

    /**
     * Because we have an array of rename maps (one per thread) in the CPU,
     * it's awkward to initialize this object via the constructor.
     * Instead, this method is used for initialization.
     */
    void init(unsigned size, SimpleFreeList *_freeList, RegIndex _zeroReg);

    /**
     * Pair of a physical register and a physical register.  Used to
     * return the physical register that a logical register has been
     * renamed to, and the previous physical register that the same
     * logical register was previously mapped to.
     */
    typedef std::pair<PhysRegIdPtr, PhysRegIdPtr> RenameInfoSec;
    
    struct RenameInfoExt{
        RenameInfoExt(PhysRegIdPtr _newPhysReg,
                      PhysRegIdPtr _prevPhysReg,
                      TheISA::PCState _prevpc,
                      bool _prevparkBit)
            : newPhysReg(_newPhysReg), prevPhysReg(_prevPhysReg),
              prevpc(_prevpc), prevparkBit(_prevparkBit)
        {
        }
        RenameInfoExt(){
            newPhysReg = nullptr;
            prevPhysReg = nullptr;
            prevpc = 0;
            prevparkBit=false;
        }

        /** The new physical register that the arch. register is renamed to. */
        PhysRegIdPtr newPhysReg;
        /** The old physical register that the arch. register was renamed to.
         */
        PhysRegIdPtr prevPhysReg;
        TheISA::PCState prevpc;
        bool prevparkBit;
    };
    //11typedef typename RenameInfoExt RenameInfoExt;



    /**
     * Tell rename map to get a new free physical register to remap
     * the specified architectural register.
     * @param arch_reg The architectural register to remap.
     * @return A RenameInfo pair indicating both the new and previous
     * physical registers.
     */
    RenameInfoExt rename(const RegId& arch_reg,TheISA::PCState pc);
    RenameInfoExt renameBeforePark(const RegId& arch_reg,TheISA::PCState pc);
    /**
     * Look up the physical register mapped to an architectural register.
     * @param arch_reg The architectural register to look up.
     * @return The physical register it is currently mapped to.
     */
    PhysRegIdPtr lookupSec(const RegId& arch_reg) const
    {
        assert(arch_reg.flatIndex() <= secmap.size());
         
        return secmap[arch_reg.flatIndex()];
    }

    PhysRegIdPtr lookupExt(const RegId& arch_reg) const
    {
        assert(arch_reg.flatIndex() <= extmap.size());

        return extmap[arch_reg.flatIndex()].preg;
    }


    /**
     * Update rename map with a specific mapping.  Generally used to
     * roll back to old mappings on a squash.
     * @param arch_reg The architectural register to remap.
     * @param phys_reg The physical register to remap it to.
     */
    void setEntrySec(const RegId& arch_reg, PhysRegIdPtr phys_reg)
    {
        assert(arch_reg.flatIndex() <= secmap.size());
        
        secmap[arch_reg.flatIndex()] = phys_reg;
    }
    
 
    void setEntryExt(const RegId& arch_reg, PhysRegIdPtr phys_reg,TheISA::PCState pc,bool parkBit)
    {
        assert(arch_reg.flatIndex() <= extmap.size());

        extmap[arch_reg.flatIndex()].preg = phys_reg;
        extmap[arch_reg.flatIndex()].pc = pc;
        extmap[arch_reg.flatIndex()].parkBit = parkBit;

    }

    void setEntryExtNULL(const RegId& arch_reg, TheISA::PCState pc,bool parkBit)
    {
        assert(arch_reg.flatIndex() <= extmap.size());

        extmap[arch_reg.flatIndex()].preg = NULL;
        extmap[arch_reg.flatIndex()].pc = pc;
        extmap[arch_reg.flatIndex()].parkBit = parkBit;

    }


    /** Return the number of free entries on the associated free list. */
    unsigned numFreeEntries() const { return freeList->numFreeRegs(); }

    /** Forward begin/cbegin to the map. */
    /** @{ */
    sec_iterator begin() { return secmap.begin(); }
    sec_const_iterator begin() const { return secmap.begin(); }
    sec_const_iterator cbegin() const { return secmap.cbegin(); }
    /** @} */

    /** Forward end/cend to the map. */
    /** @{ */
    sec_iterator end() { return secmap.end(); }
    sec_const_iterator end() const { return secmap.end(); }
    sec_const_iterator cend() const { return secmap.cend(); }
    /** @} */


    /** for second RAT Forward begin/cbegin to the map. */
    /** @{ */
    ext_iterator ext_begin() { return extmap.begin(); }
    ext_const_iterator ext_begin() const { return extmap.begin(); }
    ext_const_iterator ext_cbegin() const { return extmap.cbegin(); }
    /** @} */

    /** for second rat Forward end/cend to the map. */
    /** @{ */
    ext_iterator ext_end() { return extmap.end(); }
    ext_const_iterator ext_end() const { return extmap.end(); }
    ext_const_iterator ext_cend() const { return extmap.cend(); }
    /** @} */
    

    TheISA::PCState getSourceInstPC(const RegId& arch_reg) const
    {
        assert(arch_reg.flatIndex() <= extmap.size());
        return extmap[arch_reg.flatIndex()].pc;
    }

    RenameInfoSec renameWakeUp(const RegId& arch_reg);
    
    bool lookupParkBit(const RegId& arch_reg) const
    {
        assert(arch_reg.flatIndex() <= extmap.size());
        assert(arch_reg.flatIndex() <= secmap.size());

        return extmap[arch_reg.flatIndex()].parkBit;
        
    }

    bool setParkBit(const RegId& arch_reg) 
    {
        assert(arch_reg.flatIndex() <= extmap.size());
        assert(arch_reg.flatIndex() <= secmap.size());

        extmap[arch_reg.flatIndex()].parkBit = true;
        return true;

    }

    bool setPC(const RegId& arch_reg,TheISA::PCState pc) 
    {
        assert(arch_reg.flatIndex() <= extmap.size());
        assert(arch_reg.flatIndex() <= secmap.size());

        extmap[arch_reg.flatIndex()].pc = pc;
        return true;

    }

};

/**
 * Unified register rename map for all classes of registers.  Wraps a
 * set of class-specific rename maps.  Methods that do not specify a
 * register class (e.g., rename()) take register ids,
 * while methods that do specify a register class (e.g., renameInt())
 * take register indices.
 */
class UnifiedRenameMap
{
  private:
    static constexpr uint32_t NVecElems = TheISA::NumVecElemPerVecReg;
    using VecReg = TheISA::VecReg;
    using VecPredReg = TheISA::VecPredReg;

    /** The integer register rename map */
    SimpleRenameMap intMap;

    /** The floating-point register rename map */
    SimpleRenameMap floatMap;

    /** The condition-code register rename map */
    SimpleRenameMap ccMap;

    /** The vector register rename map */
    SimpleRenameMap vecMap;

    /** The vector element register rename map */
    SimpleRenameMap vecElemMap;

    /** The predicate register rename map */
    SimpleRenameMap predMap;

    using VecMode = Enums::VecRegRenameMode;
    VecMode vecMode;

    /**
     * The register file object is used only to get PhysRegIdPtr
     * on MiscRegs, as they are stored in it.
     */
    PhysRegFile *regFile;

  public:

    typedef SimpleRenameMap::RenameInfoExt RenameInfoExt;
    typedef SimpleRenameMap::RenameInfoSec RenameInfoSec;

    /** Default constructor.  init() must be called prior to use. */
    UnifiedRenameMap() : regFile(nullptr) {};

    /** Destructor. */
    ~UnifiedRenameMap() {};

    /** Initializes rename map with given parameters. */
    void init(PhysRegFile *_regFile,
              RegIndex _intZeroReg,
              RegIndex _floatZeroReg,
              UnifiedFreeList *freeList,
              VecMode _mode);

    /**
     * Tell rename map to get a new free physical register to remap
     * the specified architectural register. This version takes a
     * RegId and reads the  appropriate class-specific rename table.
     * @param arch_reg The architectural register id to remap.
     * @return A RenameInfo pair indicating both the new and previous
     * physical registers.
     */
    RenameInfoExt rename(const RegId& arch_reg,TheISA::PCState pc)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.rename(arch_reg,pc);
          case FloatRegClass:
            return floatMap.rename(arch_reg,pc);
          case VecRegClass:
            assert(vecMode == Enums::Full);
            return vecMap.rename(arch_reg,pc);
          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return vecElemMap.rename(arch_reg,pc);
          case VecPredRegClass:
            return predMap.rename(arch_reg,pc);
          case CCRegClass:
            return ccMap.rename(arch_reg,pc);
          case MiscRegClass:
            {
            // misc regs aren't really renamed, just remapped
            PhysRegIdPtr phys_reg = lookupExt(arch_reg);
            TheISA::PCState pc_prev = getSourceInstPC(arch_reg);
            bool prev_park_bit = lookupParkBit(arch_reg);
            // Set the new register to the previous one to keep the same
            // mapping throughout the execution.
            return RenameInfoExt(phys_reg, phys_reg,pc_prev,prev_park_bit);
            }

          default:
            panic("rename rename(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }

    RenameInfoExt renameBeforePark(const RegId& arch_reg,TheISA::PCState pc)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.renameBeforePark(arch_reg,pc);
          case FloatRegClass:
            return floatMap.renameBeforePark(arch_reg,pc);
          case VecRegClass:
            assert(vecMode == Enums::Full);
            return vecMap.renameBeforePark(arch_reg,pc);
          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return vecElemMap.renameBeforePark(arch_reg,pc);
          case VecPredRegClass:
            return predMap.renameBeforePark(arch_reg,pc);
          case CCRegClass:
            return ccMap.renameBeforePark(arch_reg,pc);
          case MiscRegClass:
            {
            // misc regs aren't really renamed, just remapped
            PhysRegIdPtr phys_reg = lookupExt(arch_reg);
            TheISA::PCState pc_prev = getSourceInstPC(arch_reg);
            bool prev_park_bit = lookupParkBit(arch_reg);
            // Set the new register to the previous one to keep the same
            // mapping throughout the execution.
            return RenameInfoExt(phys_reg, phys_reg,pc_prev,prev_park_bit);
            }

          default:
            panic("rename renameBefore(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }


    /**
     * Look up the physical register mapped to an architectural register.
     * This version takes a flattened architectural register id
     * and calls the appropriate class-specific rename table.
     * @param arch_reg The architectural register to look up.
     * @return The physical register it is currently mapped to.
     */
    PhysRegIdPtr lookupExt(const RegId& arch_reg) const
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.lookupExt(arch_reg);

          case FloatRegClass:
            return  floatMap.lookupExt(arch_reg);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.lookupExt(arch_reg);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.lookupExt(arch_reg);

          case VecPredRegClass:
            return predMap.lookupExt(arch_reg);

          case CCRegClass:
            return ccMap.lookupExt(arch_reg);

          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return regFile->getMiscRegId(arch_reg.flatIndex());

          default:
            panic("rename lookup(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }


    PhysRegIdPtr lookupSec(const RegId& arch_reg) const
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.lookupSec(arch_reg);

          case FloatRegClass:
            return  floatMap.lookupSec(arch_reg);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.lookupSec(arch_reg);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.lookupSec(arch_reg);

          case VecPredRegClass:
            return predMap.lookupSec(arch_reg);

          case CCRegClass:
            return ccMap.lookupSec(arch_reg);

          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return regFile->getMiscRegId(arch_reg.flatIndex());

          default:
            panic("rename lookup(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }


    /**
     * Update rename map with a specific mapping.  Generally used to
     * roll back to old mappings on a squash.  This version takes a
     * flattened architectural register id and calls the
     * appropriate class-specific rename table.
     * @param arch_reg The architectural register to remap.
     * @param phys_reg The physical register to remap it to.
     */
    void setEntrySec(const RegId& arch_reg, PhysRegIdPtr phys_reg)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            assert(phys_reg->isIntPhysReg());
            return intMap.setEntrySec(arch_reg, phys_reg);

          case FloatRegClass:
            assert(phys_reg->isFloatPhysReg());
            return floatMap.setEntrySec(arch_reg, phys_reg);

          case VecRegClass:
            assert(phys_reg->isVectorPhysReg());
            assert(vecMode == Enums::Full);
            return vecMap.setEntrySec(arch_reg, phys_reg);

          case VecElemClass:
            assert(phys_reg->isVectorPhysElem());
            assert(vecMode == Enums::Elem);
            return vecElemMap.setEntrySec(arch_reg, phys_reg);

          case VecPredRegClass:
            assert(phys_reg->isVecPredPhysReg());
            return predMap.setEntrySec(arch_reg, phys_reg);

          case CCRegClass:
            assert(phys_reg->isCCPhysReg());
            return ccMap.setEntrySec(arch_reg, phys_reg);

          case MiscRegClass:
            // Misc registers do not actually rename, so don't change
            // their mappings.  We end up here when a commit or squash
            // tries to update or undo a hardwired misc reg nmapping,
            // which should always be setting it to what it already is.
            assert(phys_reg == lookupSec(arch_reg));
            return;

          default:
            panic("rename setEntry(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
    


    void setEntryExt(const RegId& arch_reg, PhysRegIdPtr phys_reg,TheISA::PCState pc,bool parkBit)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            assert(phys_reg->isIntPhysReg());
            return intMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case FloatRegClass:
            assert(phys_reg->isFloatPhysReg());
            return floatMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case VecRegClass:
            assert(phys_reg->isVectorPhysReg());
            assert(vecMode == Enums::Full);
            return vecMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case VecElemClass:
            assert(phys_reg->isVectorPhysElem());
            assert(vecMode == Enums::Elem);
            return vecElemMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case VecPredRegClass:
            assert(phys_reg->isVecPredPhysReg());
            return predMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case CCRegClass:
            assert(phys_reg->isCCPhysReg());
            return ccMap.setEntryExt(arch_reg, phys_reg,pc,parkBit);

          case MiscRegClass:
            // Misc registers do not actually rename, so don't change
            // their mappings.  We end up here when a commit or squash
            // tries to update or undo a hardwired misc reg nmapping,
            // which should always be setting it to what it already is.
            assert(phys_reg == lookupExt(arch_reg));
            return;

          default:
            panic("rename setEntry(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
   
    void setEntryExtNULL(const RegId& arch_reg, TheISA::PCState pc,bool parkBit)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case FloatRegClass:
            return floatMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return vecMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return vecElemMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case VecPredRegClass:
            return predMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case CCRegClass:
            return ccMap.setEntryExtNULL(arch_reg, pc,parkBit);

          case MiscRegClass:
            // Misc registers do not actually rename, so don't change
            // their mappings.  We end up here when a commit or squash
            // tries to update or undo a hardwired misc reg nmapping,
            // which should always be setting it to what it already is.
            return;

          default:
            panic("rename setEntry(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
    /**
     * Return the minimum number of free entries across all of the
     * register classes.  The minimum is used so we guarantee that
     * this number of entries is available regardless of which class
     * of registers is requested.
     */
    unsigned numFreeEntries() const
    {
        return std::min(std::min(
                std::min(intMap.numFreeEntries(), floatMap.numFreeEntries()),
                vecMode == Enums::Full ? vecMap.numFreeEntries()
                                    : vecElemMap.numFreeEntries()),
                predMap.numFreeEntries());
    }

    unsigned numFreeIntEntries() const { return intMap.numFreeEntries(); }
    unsigned numFreeFloatEntries() const { return floatMap.numFreeEntries(); }
    unsigned numFreeVecEntries() const
    {
        return vecMode == Enums::Full
                ? vecMap.numFreeEntries()
                : vecElemMap.numFreeEntries();
    }
    unsigned numFreePredEntries() const { return predMap.numFreeEntries(); }
    unsigned numFreeCCEntries() const { return ccMap.numFreeEntries(); }

    /**
     * Return whether there are enough registers to serve the request.
     */
    bool canRename(uint32_t intRegs, uint32_t floatRegs, uint32_t vectorRegs,
                   uint32_t vecElemRegs, uint32_t vecPredRegs,
                   uint32_t ccRegs) const
    {
        return intRegs <= intMap.numFreeEntries() &&
            floatRegs <= floatMap.numFreeEntries() &&
            vectorRegs <= vecMap.numFreeEntries() &&
            vecElemRegs <= vecElemMap.numFreeEntries() &&
            vecPredRegs <= predMap.numFreeEntries() &&
            ccRegs <= ccMap.numFreeEntries();
    }
    /**
     * Set vector mode to Full or Elem.
     * Ignore 'silent' modifications.
     *
     * @param newVecMode new vector renaming mode
     */
    void switchMode(VecMode newVecMode);

    /**
     * Switch freeList of registers from Full to Elem or vicevers
     * depending on vecMode (vector renaming mode).
     */
    void switchFreeList(UnifiedFreeList* freeList);
    
    /**get pc from source reg using RAT*/
    TheISA::PCState getSourceInstPC(const RegId& arch_reg) const
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.getSourceInstPC(arch_reg);

          case FloatRegClass:
            return  floatMap.getSourceInstPC(arch_reg);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.getSourceInstPC(arch_reg);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.getSourceInstPC(arch_reg);

          case VecPredRegClass:
            return predMap.getSourceInstPC(arch_reg);

          case CCRegClass:
            return ccMap.getSourceInstPC(arch_reg);
          
          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return 0;

          default:
            panic("rename getSourceInstPC(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
    
    RenameInfoSec renameWakeUp(const RegId& arch_reg)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.renameWakeUp(arch_reg);
          case FloatRegClass:
            return floatMap.renameWakeUp(arch_reg);
          case VecRegClass:
            assert(vecMode == Enums::Full);
            return vecMap.renameWakeUp(arch_reg);
          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return vecElemMap.renameWakeUp(arch_reg);
          case VecPredRegClass:
            return predMap.renameWakeUp(arch_reg);
          case CCRegClass:
            return ccMap.renameWakeUp(arch_reg);
          case MiscRegClass:
            {
               // misc regs aren't really renamed, just remapped
                PhysRegIdPtr phys_reg = lookupSec(arch_reg);
                // Set the new register to the previous one to keep the same
                // mapping throughout the execution.
                return RenameInfoSec(phys_reg, phys_reg);
            }

          default:
            panic("rename rename(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
   
    bool lookupParkBit(const RegId& arch_reg) const
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:

            return intMap.lookupParkBit(arch_reg);

          case FloatRegClass:
            return  floatMap.lookupParkBit(arch_reg);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.lookupParkBit(arch_reg);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.lookupParkBit(arch_reg);

          case VecPredRegClass:
            return predMap.lookupParkBit(arch_reg);

          case CCRegClass:
            return ccMap.lookupParkBit(arch_reg);

          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return false;

          default:
            panic("rename lookupParkBit(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }
  
    bool setParkBit(const RegId& arch_reg) 
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.setParkBit(arch_reg);

          case FloatRegClass:
            return  floatMap.setParkBit(arch_reg);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.setParkBit(arch_reg);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.setParkBit(arch_reg);

          case VecPredRegClass:
            return predMap.setParkBit(arch_reg);

          case CCRegClass:
            return ccMap.setParkBit(arch_reg);

          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return false;

          default:
            panic("rename lookupParkBit(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }

    bool setPC(const RegId& arch_reg,TheISA::PCState pc)
    {
        switch (arch_reg.classValue()) {
          case IntRegClass:
            return intMap.setPC(arch_reg,pc);

          case FloatRegClass:
            return  floatMap.setPC(arch_reg,pc);

          case VecRegClass:
            assert(vecMode == Enums::Full);
            return  vecMap.setPC(arch_reg,pc);

          case VecElemClass:
            assert(vecMode == Enums::Elem);
            return  vecElemMap.setPC(arch_reg,pc);

          case VecPredRegClass:
            return predMap.setPC(arch_reg,pc);

          case CCRegClass:
            return ccMap.setPC(arch_reg,pc);

          case MiscRegClass:
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return false;

          default:
            panic("rename setPC(): unknown reg class %s\n",
                  arch_reg.className());
        }
    }



};

#endif //__CPU_O3_RENAME_MAP_HH__

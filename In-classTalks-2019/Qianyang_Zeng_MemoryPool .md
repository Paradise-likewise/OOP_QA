# Memory pool

Qianyang Zeng

## Definition

**Memory pools**, also called **fixed-size blocks allocation**, is used for memory management.

It is designed to implement dynamic memory allocation, like C's `malloc` of C++'s `operator new`. 

<font size = 4>But different from `malloc` and `new`, each time it allocates **a fixed size of memory block** instead of inconstant size of memory.</font>

It allows users to **avoid memory fragmentation**, to improve the efficiency and memory utilization.

## Why memory pool?

The default memory allocation functions, `malloc` and `operator new`, allocate variable block sizes of memory. In a real time system, they are less efficient because they produce great amount of ***"memory fragments"*** scattering in all parts of memory, and it causes the available memory to shrink. **It is often called a kind of "memory leaks"**.

Also, by calling free/delete, the system may need to merge free memory blocks, which takes extra time and cause lower efficiency.

As a result, the default memory allocation functions are not recommendable to used in **a real time system**, since it needs to **allocate and free memory repeatedly**. A more efficient solution is pre-allocating a number of memory blocks with the same size called the **memory pool**.

## Example Structure

4 important parts will be introduced.

### 1. `Memorypool`

```c++
class MemoryPool
{
private:
    MemoryBlock*	pBlock;
    USHORT			nUnitSize;
    USHORT			nInitSize;
    USHORT			nGrowSize;
public:
    MemoryPool(USHORT nUnitSize, 
               USHORT nInitSize = 1024, 
               USHORT nGrowSize = 256);
    ~MemoryPool();
    void* Alloc();
    void Free(void* p);
};

MemoryPool::Memorypool(USHORT nUnitSize, 
               USHORT nInitSize, 
               USHORT nGrowSize)
{
    pBlock		= NULL;
    nInitSize	=_nInitSize;
    nGrowSize	=_nGrowSize;
    
    if(_nUnitSize > 4)
        nUnitSize = (_nUnitSize + (MEMPOOL_ALIGNMENT-1))&~(MEMPOOL_ALIGNMENT-1);
    else if(_nUnitSize <= 2) nUnitSize = 2;
    else nUnitSize = 4;
}
```

A `MemoryPool`  consists of a series of fixed size `MemoryBlock`s. 

Each `MemoryBlock` contains a fixed number and size of `MemoryUnit`s. 

Each unit is a fixed size of memory. 

And `MemoryPool` contains the message that how many units does a `MemoryBlock` contains. The message is the **4 data members** in class `MemoryPool`:

​				`	pBlock`: pointer that points to the first `MemoryBlock`

​				`	nUnitSize`: size of each memory unit (see the second part)

​				`	nInitSize`: the number of memory units in first `MemoryBlock`

​				`nGrowSize`: the number of memory units in other `MemoryBlock`s



A `MemoryPool` contains a series of `MemoryBlock`s (only have one `MemoryBlock` pointed by `pBlock` when initiated), and each `MemoryBlock` contains a number of `MemoryUnit`s. 



### 2. `MemoryBlock`

When a `MemoryPool` object is first generated, the system only allocates one `MemoryBlock ` object, pointed by `pBlock`  pointer in `MemoryPool`. Later, with more demands for memory, the first `MemoryBlock` is not enough, so the second `MemoryBlock` is allocated. All these `MemoryBlock`s together are linked through pointers `pNext`.

```c++
struct MemoryBlock
{
    USHORT			nSize;
    USHORT			nFree;
    USHORT			nFirst;
    USHORT			nDummyAlign1;
    MemoryBlock*	pNext;
    char			aData[1];
    
    static void* operator new(size_t, USHORT nTypes, USHORT nUnitSize)
    {
        return ::operator new(sizeof(MemoryBlock) + nTypes*nUnitSize);
    }
    static void operator delete(void *p, size_t)
    {
        ::operator delete(p);
    }
    
    MeomoryBlock (USHORT nTypes = 1, USHORT nUnitSize = 0);
    ~MemoryBlock() {}
};

MemoryBlock::MemoryBlock (USHORT nTypes, USHORT nUnitSize):
	nSize (nTypes*nUnitSize),
	nFree (nTypes - 1),
	nFirst (1),
	pNext (0)
{
    char* pData = aData;
        for(USHORT i = 1; i<nTypes; i++)
        {
            *reinterpret_cast<USHORT*>(pData) = i;
            pData += nUnitSize;
        }
}
```

`nSize`: size of this `MemoryBlock`.

`nFree`: number of available `MemoryUnit`.

`nFirst`: index of the next available `MemoryUnit`.

`pNext`: pointing to the next `MemoryBlock`.

### 3. `Alloc()`

`Alloc()` is the member function in `MemoryPool`, used to allocate a whole `MemoryBlock` each time. The simple code is as follow.

```c++
void* MemoryPool::Alloc()
{
    if(!pBloc)
    {
        ...
    }
    
    MemoryBloc* pMyBlock = pBlock;
    while (pMyBlock && !pMyBlock->nFree)
        pMyBlock = pMyBlock->pNext;
    
    if(pMyBlock)
    {
        char* pFree = pMyBlock->aData+(pMyBlock->nFirst*nUnitSize);
        pMyBlock->nFirst = *((USHORT*)pFree);
        
        pMyBlock->nFree--;
        return (void*)pFree;
    }
    else
    {
        if(!nGrowSize)
            return NULL;
        
        pMyBlock = new(nGrowSize, nUnitSize) FixedMemBlock(nGrowSize, nUnitSize);
        if(!pMyBlock)
            return NULL;
        
        pMyBlock->pNext = pBlock;
        pBlock = pMyBlock;
        
        return (void*)(pMyBlock->aData);
    }
}
```



### 4. `Free(void* pFree)`

Pass a pointer pointing to an after-use memory as parameter.

```c++
void MemoryPool::Free( void* pFree )
{
    ……
 
    MemoryBlock* pMyBlock = pBlock;
 
    while ( ((ULONG)pMyBlock->aData > (ULONG)pFree) ||
         ((ULONG)pFree >= ((ULONG)pMyBlock->aData + pMyBlock->nSize)) )
    {
         ……
    }

    pMyBlock->nFree++;
    *((USHORT*)pFree) = pMyBlock->nFirst;
    pMyBlock->nFirst = (USHORT)(((ULONG)pFree-(ULONG)(pBlock->aData)) / nUnitSize);
 
    if (pMyBlock->nFree*nUnitSize == pMyBlock->nSize )
    {
        ……
    }
    else
    {
        ……
    }
}
```

## How to use Memory Pool?

Memory Pool mainly has two interface functions: `Alloc()` and `Free()`.  `Alloc()`  returns a memory and `Free()` delete the memory and give it back to the Memory Pool. 

So user needs to declare a static Memory Pool and make all the memory allocations managed by this static `MemoryPool` object. As a result, operator new and delete needs overloading.
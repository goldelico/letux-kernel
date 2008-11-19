/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@file: sdlist.h

@abstract: OS independent list functions

@notice: Copyright (c), 2004-2006 Atheros Communications, Inc.


 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation;
 *
 *  Software distributed under the License is distributed on an "AS
 *  IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  rights and limitations under the License.
 *
 *  Portions of this code were developed with information supplied from the
 *  SD Card Association Simplified Specifications. The following conditions and disclaimers may apply:
 *
 *   The following conditions apply to the release of the SD simplified specification (�Simplified
 *   Specification�) by the SD Card Association. The Simplified Specification is a subset of the complete
 *   SD Specification which is owned by the SD Card Association. This Simplified Specification is provided
 *   on a non-confidential basis subject to the disclaimers below. Any implementation of the Simplified
 *   Specification may require a license from the SD Card Association or other third parties.
 *   Disclaimers:
 *   The information contained in the Simplified Specification is presented only as a standard
 *   specification for SD Cards and SD Host/Ancillary products and is provided "AS-IS" without any
 *   representations or warranties of any kind. No responsibility is assumed by the SD Card Association for
 *   any damages, any infringements of patents or other right of the SD Card Association or any third
 *   parties, which may result from its use. No license is granted by implication, estoppel or otherwise
 *   under any patent or other rights of the SD Card Association or any third party. Nothing herein shall
 *   be construed as an obligation by the SD Card Association to disclose or distribute any technical
 *   information, know-how or other confidential information to any third party.
 *
 *
 *  The initial developers of the original code are Seung Yi and Paul Lever
 *
 *  sdio@atheros.com
 *
 *

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#ifndef __SDLIST_H___
#define __SDLIST_H___

/* list functions */
/* pointers for the list */
typedef struct _SDLIST {
    struct _SDLIST *pPrev;
    struct _SDLIST *pNext;
}SDLIST, *PSDLIST;
/*
 * SDLIST_INIT , circular list
*/
#define SDLIST_INIT(pList)\
    {(pList)->pPrev = pList; (pList)->pNext = pList;}
#define SDLIST_INIT_DECLARE(List)\
    SDLIST List =   {&List, &List}


#define SDLIST_IS_EMPTY(pList) (((pList)->pPrev == (pList)) && ((pList)->pNext == (pList)))
#define SDLIST_GET_ITEM_AT_HEAD(pList) (pList)->pNext
#define SDLIST_GET_ITEM_AT_TAIL(pList) (pList)->pPrev
/*
 * SDITERATE_OVER_LIST pStart is the list, pTemp is a temp list member
 * NOT: do not use this function if the items in the list are deleted inside the
 * iteration loop
*/
#define SDITERATE_OVER_LIST(pStart, pTemp) \
    for((pTemp) =(pStart)->pNext; pTemp != (pStart); (pTemp) = (pTemp)->pNext)


/* safe iterate macro that allows the item to be removed from the list
 * the iteration continues to the next item in the list
 */
#define SDITERATE_OVER_LIST_ALLOW_REMOVE(pStart,pItem,st,offset)  \
{                                                       \
    PSDLIST  pTemp;                                     \
    pTemp = (pStart)->pNext;                            \
    while (pTemp != (pStart)) {                         \
        (pItem) = CONTAINING_STRUCT(pTemp,st,offset);   \
         pTemp = pTemp->pNext;                          \

#define SDITERATE_END }}

/*
 * SDListInsertTail - insert pAdd to the end of the list
*/
static INLINE PSDLIST SDListInsertTail(PSDLIST pList, PSDLIST pAdd) {
        /* this assert catches when an item is added twice */
    DBG_ASSERT(pAdd->pNext != pList);
        /* insert at tail */
    pAdd->pPrev = pList->pPrev;
    pAdd->pNext = pList;
    pList->pPrev->pNext = pAdd;
    pList->pPrev = pAdd;
    return pAdd;
}

/*
 * SDListInsertHead - insert pAdd into the head of the list
*/
static INLINE PSDLIST SDListInsertHead(PSDLIST pList, PSDLIST pAdd) {
        /* this assert catches when an item is added twice */
    DBG_ASSERT(pAdd->pPrev != pList);
        /* insert at head */
    pAdd->pPrev = pList;
    pAdd->pNext = pList->pNext;
    pList->pNext->pPrev = pAdd;
    pList->pNext = pAdd;
    return pAdd;
}

#define SDListAdd(pList,pItem) SDListInsertHead((pList),(pItem))
/*
 * SDListRemove - remove pDel from list
*/
static INLINE PSDLIST SDListRemove(PSDLIST pDel) {
    pDel->pNext->pPrev = pDel->pPrev;
    pDel->pPrev->pNext = pDel->pNext;
        /* point back to itself just to be safe, incase remove is called again */
    pDel->pNext = pDel;
    pDel->pPrev = pDel;
    return pDel;
}

/*
 * SDListRemoveItemFromHead - get a list item from the head
*/
static INLINE PSDLIST SDListRemoveItemFromHead(PSDLIST pList) {
    PSDLIST pItem = NULL;
    if (pList->pNext != pList) {
        pItem = pList->pNext;
            /* remove the first item from head */
        SDListRemove(pItem);
    }
    return pItem;
}
#endif /* __SDLIST_H___ */

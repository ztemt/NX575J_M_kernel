#include "precomp.h"
#include "gl_kal.h"

struct COMMAND {
	UINT_8 ucCID;
	BOOLEAN fgSetQuery;
	BOOLEAN fgNeedResp;
	UINT_8 ucCmdSeqNum;
};

struct SECURITY_FRAME {
	UINT_16 u2EthType;
	UINT_16 u2Reserved;
};

struct MGMT_FRAME {
	UINT_16 u2FrameCtl;
	UINT_16 u2DurationID;
};

typedef struct _TC_RES_RELEASE_ENTRY {
	UINT_64 u8RelaseTime;
	UINT_32 u4RelCID;
	UINT_8	ucTc4RelCnt;
	UINT_8	ucAvailableTc4;
} TC_RES_RELEASE_ENTRY, *P_TC_RES_RELEASE_ENTRY;

typedef struct _CMD_TRACE_ENTRY {
	UINT_64 u8TxTime;
	COMMAND_TYPE eCmdType;
	union {
		struct COMMAND rCmd;
		struct SECURITY_FRAME rSecFrame;
		struct MGMT_FRAME rMgmtFrame;
	} u;
} CMD_TRACE_ENTRY, *P_CMD_TRACE_ENTRY;

typedef struct _COMMAND_ENTRY {
	UINT_64 u8TxTime;
	UINT_64 u8ReadFwTime;
	UINT_32 u4ReadFwValue;
	UINT_32 u4RelCID;
	UINT_16 u2Counter;
	struct COMMAND rCmd;
} COMMAND_ENTRY, *P_COMMAND_ENTRY;

typedef struct _SCAN_HIF_DESC_RECORD {
	P_HIF_TX_DESC_T pTxDescScanWriteBefore;
	P_HIF_TX_DESC_T pTxDescScanWriteDone;
	UINT_64 u8ScanWriteBeforeTime;
	UINT_64 u8ScanWriteDoneTime;
	UINT_32 aucFreeBufCntScanWriteBefore;
	UINT_32 aucFreeBufCntScanWriteDone;
} SCAN_HIF_DESC_RECORD, *P_SCAN_HIF_DESC_RECORD;

#define TC_RELEASE_TRACE_BUF_MAX_NUM 100
#define TXED_CMD_TRACE_BUF_MAX_NUM 100
#define TXED_COMMAND_BUF_MAX_NUM 10

static P_TC_RES_RELEASE_ENTRY gprTcReleaseTraceBuffer;
static P_CMD_TRACE_ENTRY gprCmdTraceEntry;
static P_COMMAND_ENTRY gprCommandEntry;
static SCAN_HIF_DESC_RECORD grScanHifDescRecord;

VOID wlanDebugInit(VOID)
{
	/* debug for command/tc4 resource begin */
	gprTcReleaseTraceBuffer =
		kalMemAlloc(TC_RELEASE_TRACE_BUF_MAX_NUM * sizeof(TC_RES_RELEASE_ENTRY), PHY_MEM_TYPE);
	kalMemZero(gprTcReleaseTraceBuffer, TC_RELEASE_TRACE_BUF_MAX_NUM * sizeof(TC_RES_RELEASE_ENTRY));
	gprCmdTraceEntry = kalMemAlloc(TXED_CMD_TRACE_BUF_MAX_NUM * sizeof(CMD_TRACE_ENTRY), PHY_MEM_TYPE);
	kalMemZero(gprCmdTraceEntry, TXED_CMD_TRACE_BUF_MAX_NUM * sizeof(CMD_TRACE_ENTRY));

	gprCommandEntry = kalMemAlloc(TXED_COMMAND_BUF_MAX_NUM * sizeof(COMMAND_ENTRY), PHY_MEM_TYPE);
	kalMemZero(gprCommandEntry, TXED_COMMAND_BUF_MAX_NUM * sizeof(COMMAND_ENTRY));
	/* debug for command/tc4 resource end */

	/*debug for scan request tx_description begin*/
	grScanHifDescRecord.pTxDescScanWriteBefore = kalMemAlloc(NIC_TX_BUFF_COUNT_TC4 * sizeof(HIF_TX_DESC_T)
		, VIR_MEM_TYPE);
	grScanHifDescRecord.aucFreeBufCntScanWriteBefore = 0;
	grScanHifDescRecord.pTxDescScanWriteDone = kalMemAlloc(NIC_TX_BUFF_COUNT_TC4 * sizeof(HIF_TX_DESC_T)
	, VIR_MEM_TYPE);
	grScanHifDescRecord.aucFreeBufCntScanWriteDone = 0;
	/*debug for scan request tx_description end*/

}

VOID wlanDebugUninit(VOID)
{
	/* debug for command/tc4 resource begin */
	kalMemFree(gprTcReleaseTraceBuffer, PHY_MEM_TYPE,
			TC_RELEASE_TRACE_BUF_MAX_NUM * sizeof(TC_RES_RELEASE_ENTRY));
	kalMemFree(gprCmdTraceEntry, PHY_MEM_TYPE, TXED_CMD_TRACE_BUF_MAX_NUM * sizeof(CMD_TRACE_ENTRY));
	kalMemFree(gprCommandEntry, PHY_MEM_TYPE, TXED_COMMAND_BUF_MAX_NUM * sizeof(COMMAND_ENTRY));
	/* debug for command/tc4 resource end */

	/*debug for scan request tx_description begin*/
	kalMemFree(grScanHifDescRecord.pTxDescScanWriteBefore
	, VIR_MEM_TYPE, NIC_TX_BUFF_COUNT_TC4 * sizeof(HIF_TX_DESC_T));
	grScanHifDescRecord.aucFreeBufCntScanWriteBefore = 0;
	kalMemFree(grScanHifDescRecord.pTxDescScanWriteDone
	, VIR_MEM_TYPE, NIC_TX_BUFF_COUNT_TC4 * sizeof(HIF_TX_DESC_T));
	grScanHifDescRecord.aucFreeBufCntScanWriteDone = 0;
	/*debug for scan request tx_description end*/
}

VOID wlanDebugHifDescriptorRecord(P_ADAPTER_T prAdapter, ENUM_AMPDU_TYPE type
	, ENUM_DEBUG_TRAFFIC_CLASS_INDEX_T tcIndex, PUINT_8 pucBuffer)
{
	UINT_32 i;
	UINT_32 u4Offset;
	UINT_32 u4StartAddr;
	P_HIF_TX_DESC_T prTxDesc;
	P_HIF_RX_DESC_T prRxDesc;
	UINT_32 u4TcCount;

	if (pucBuffer == NULL) {
		DBGLOG(TX, ERROR, "wlanDebugHifDescriptorRecord pucBuffer is Null !");
		return;
	}


	if (type == MTK_AMPDU_TX_DESC) {

		if (tcIndex == DEBUG_TC0_INDEX) {
			u4TcCount = NIC_TX_INIT_BUFF_COUNT_TC0;
			u4StartAddr = AP_MCU_TX_DESC_ADDR;
			u4Offset = AP_MCU_BANK_OFFSET;
		} else if (tcIndex == DEBUG_TC4_INDEX) {
			u4TcCount = NIC_TX_BUFF_COUNT_TC4;
			u4StartAddr = AP_MCU_TC_INDEX_4_ADDR;
			u4Offset = AP_MCU_TC_INDEX_4_OFFSET;
		} else {
			DBGLOG(TX, ERROR, "Type :%d TC_INDEX :%d don't support !", type , tcIndex);
			return;
		}

		prTxDesc = (P_HIF_TX_DESC_T)pucBuffer;
		for (i = 0; i < u4TcCount ; i++)
			HAL_GET_APMCU_MEM(prAdapter, u4StartAddr, u4Offset, i, (PUINT_8) &prTxDesc[i]
				, sizeof(HIF_TX_DESC_T));


	} else if (type == MTK_AMPDU_RX_DESC) {

		if (tcIndex == DEBUG_TC0_INDEX) {
			u4TcCount = NIC_TX_INIT_BUFF_COUNT_TC0;
			u4StartAddr = AP_MCU_RX_DESC_ADDR;
			u4Offset = AP_MCU_BANK_OFFSET;
		} else {
			DBGLOG(RX, ERROR, "Type :%d TC_INDEX :%d don't support !", type, tcIndex);
			return;
		}

		prRxDesc = (P_HIF_RX_DESC_T)pucBuffer;
		for (i = 0; i < u4TcCount ; i++)
			HAL_GET_APMCU_MEM(prAdapter, u4StartAddr, u4Offset, i, (PUINT_8) &prRxDesc[i]
				, sizeof(HIF_RX_DESC_T));
	}

}

VOID wlanDebugHifDescriptorPrint(P_ADAPTER_T prAdapter , ENUM_AMPDU_TYPE type
	, ENUM_DEBUG_TRAFFIC_CLASS_INDEX_T tcIndex, PUINT_8 pucBuffer)
{
	UINT_32 i;
	UINT_32 u4TcCount;
	P_HIF_TX_DESC_T prTxDesc;
	P_HIF_RX_DESC_T prRxDesc;

	if (pucBuffer == NULL) {
		DBGLOG(TX, ERROR, "wlanDebugHifDescriptorDump pucBuffer is Null !");
		return;
	}

	if (type == MTK_AMPDU_TX_DESC) {
		if (tcIndex == DEBUG_TC0_INDEX)
			u4TcCount = NIC_TX_INIT_BUFF_COUNT_TC0;
		else if (tcIndex == DEBUG_TC4_INDEX)
			u4TcCount = NIC_TX_BUFF_COUNT_TC4;
		else {
			DBGLOG(TX, ERROR, "Type :%d TC_INDEX :%d don't support !", type , tcIndex);
			return;
		}

		prTxDesc = (P_HIF_TX_DESC_T)pucBuffer;
		DBGLOG(TX, INFO, "Start dump Tx_desc from APMCU\n");
		for (i = 0; i < u4TcCount ; i++) {
				DBGLOG(TX, INFO
					, "TC%d[%d]uOwn:%2x,CS:%2x,R1:%2x,ND:0x%08x,SA: 0x%08x,R2:%x\n"
					, tcIndex, i, prTxDesc[i].ucOwn, prTxDesc[i].ucDescChksum
					, prTxDesc[i].u2Rsrv1, prTxDesc[i].u4NextDesc
					, prTxDesc[i].u4BufStartAddr, prTxDesc[i].u4Rsrv2);
		}

	} else if (type == MTK_AMPDU_RX_DESC) {

		if (tcIndex == DEBUG_TC0_INDEX)
			u4TcCount = NIC_TX_INIT_BUFF_COUNT_TC0;
		else {
			DBGLOG(RX, ERROR, "Type :%d TC_INDEX :%d don't support !", type, tcIndex);
			return;
		}

		prRxDesc = (P_HIF_RX_DESC_T)pucBuffer;
		DBGLOG(RX, INFO, "Start dump rx_desc from APMCU\n");
		for (i = 0; i < u4TcCount ; i++) {
			DBGLOG(RX, INFO
				, "RX%d[%d]uOwn:%2x,CS:%2x,TO:%x,CSI:%x,ND:0x%08x,SA:0x%08x,len:%x,R1:%x\n"
				, tcIndex, i, prRxDesc[i].ucOwn , prRxDesc[i].ucDescChksum
				, prRxDesc[i].ucEtherTypeOffset, prRxDesc[i].ucChkSumInfo
				, prRxDesc[i].u4NextDesc, prRxDesc[i].u4BufStartAddr
				, prRxDesc[i].u2RxBufLen, prRxDesc[i].u2Rsrv1);

		}
	}

}

VOID wlanDebugHifDescriptorDump(P_ADAPTER_T prAdapter , ENUM_AMPDU_TYPE type
	, ENUM_DEBUG_TRAFFIC_CLASS_INDEX_T tcIndex)
{
	UINT_32 size = NIC_TX_BUFF_SUM;
	P_HIF_TX_DESC_T prTxDesc;
	P_HIF_RX_DESC_T prRxDesc;


	if (type == MTK_AMPDU_TX_DESC) {

		prTxDesc = (P_HIF_TX_DESC_T) kalMemAlloc(sizeof(HIF_TX_DESC_T) * size, VIR_MEM_TYPE);
		if (prTxDesc == NULL) {
			DBGLOG(TX, WARN, "wlanDebugHifDescriptorDump prTxDesc alloc fail!\n");
			return;
		}
		kalMemZero(prTxDesc, sizeof(HIF_TX_DESC_T) * size);
		wlanDebugHifDescriptorRecord(prAdapter, type, tcIndex, (PUINT_8)prTxDesc);
		wlanDebugHifDescriptorPrint(prAdapter, type, tcIndex, (PUINT_8)prTxDesc);
		kalMemFree(prTxDesc, VIR_MEM_TYPE, sizeof(HIF_TX_DESC_T));

	} else if (type == MTK_AMPDU_RX_DESC) {

		prRxDesc = (P_HIF_RX_DESC_T) kalMemAlloc(sizeof(HIF_RX_DESC_T) * size, VIR_MEM_TYPE);
		if (prRxDesc == NULL) {
			DBGLOG(RX, WARN, "wlanDebugHifDescriptorDump prRxDesc alloc fail!\n");
			return;
		}
		kalMemZero(prRxDesc, sizeof(HIF_RX_DESC_T) * size);
		wlanDebugHifDescriptorRecord(prAdapter, type, tcIndex, (PUINT_8)prRxDesc);
		wlanDebugHifDescriptorPrint(prAdapter, type, tcIndex, (PUINT_8)prRxDesc);
		kalMemFree(prRxDesc, VIR_MEM_TYPE, sizeof(P_HIF_RX_DESC_T));
	}


}
VOID wlanDebugScanRecord(P_ADAPTER_T prAdapter, ENUM_DBG_SCAN_T recordType)
{

	UINT_32 tcIndex = DEBUG_TC4_INDEX;
	UINT_32 type = MTK_AMPDU_TX_DESC;
	P_TX_CTRL_T pTxCtrl = &prAdapter->rTxCtrl;

	if (recordType == DBG_SCAN_WRITE_BEFORE) {
		wlanDebugHifDescriptorRecord(prAdapter, type, tcIndex
		, (PUINT_8)grScanHifDescRecord.pTxDescScanWriteBefore);
		grScanHifDescRecord.aucFreeBufCntScanWriteBefore = pTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX];
		grScanHifDescRecord.u8ScanWriteBeforeTime = sched_clock();
	} else if (recordType == DBG_SCAN_WRITE_DONE) {
		wlanDebugHifDescriptorRecord(prAdapter, type, tcIndex
		, (PUINT_8)grScanHifDescRecord.pTxDescScanWriteDone);
		grScanHifDescRecord.aucFreeBufCntScanWriteDone = pTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX];
		grScanHifDescRecord.u8ScanWriteDoneTime = sched_clock();
	}

}
VOID wlanDebugScanDump(P_ADAPTER_T prAdapter)
{
	UINT_32 tcIndex = DEBUG_TC4_INDEX;
	UINT_32 type = MTK_AMPDU_TX_DESC;

	DBGLOG(TX, INFO, "ScanReq hal write before:Time=%llu ,freeCnt=%d,dump tc4[0]~[3] desc!\n"
		, grScanHifDescRecord.u8ScanWriteBeforeTime
		, grScanHifDescRecord.aucFreeBufCntScanWriteBefore);
	wlanDebugHifDescriptorPrint(prAdapter, type, tcIndex
		, (PUINT_8)grScanHifDescRecord.pTxDescScanWriteBefore);

	DBGLOG(TX, INFO, "ScanReq hal write done:Time=%llu ,freeCnt=%d,dump tc4[0]~[3] desc!\n"
		, grScanHifDescRecord.u8ScanWriteDoneTime
		, grScanHifDescRecord.aucFreeBufCntScanWriteDone);
	wlanDebugHifDescriptorPrint(prAdapter, type, tcIndex
		, (PUINT_8)grScanHifDescRecord.pTxDescScanWriteDone);
}


VOID wlanReadFwStatus(P_ADAPTER_T prAdapter)
{
	static UINT_16 u2CurEntryCmd;
	P_COMMAND_ENTRY prCurCommand = &gprCommandEntry[u2CurEntryCmd];

	prCurCommand->u8ReadFwTime = sched_clock();
	HAL_MCR_RD(prAdapter, MCR_D2HRM2R, &prCurCommand->u4ReadFwValue);
	u2CurEntryCmd++;
	if (u2CurEntryCmd == TXED_COMMAND_BUF_MAX_NUM)
		u2CurEntryCmd = 0;
}

VOID wlanTraceTxCmd(P_ADAPTER_T prAdapter, P_CMD_INFO_T prCmd)
{
	static UINT_16 u2CurEntry;
	static UINT_16 u2CurEntryCmd;
	P_CMD_TRACE_ENTRY prCurCmd = &gprCmdTraceEntry[u2CurEntry];
	P_COMMAND_ENTRY prCurCommand = &gprCommandEntry[u2CurEntryCmd];

	prCurCmd->u8TxTime = sched_clock();
	prCurCommand->u8TxTime = prCurCmd->u8TxTime;
	prCurCmd->eCmdType = prCmd->eCmdType;
	if (prCmd->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME) {
		P_WLAN_MAC_MGMT_HEADER_T prMgmt = (P_WLAN_MAC_MGMT_HEADER_T)((P_MSDU_INFO_T)prCmd->prPacket)->prPacket;

		prCurCmd->u.rMgmtFrame.u2FrameCtl = prMgmt->u2FrameCtrl;
		prCurCmd->u.rMgmtFrame.u2DurationID = prMgmt->u2Duration;
	} else if (prCmd->eCmdType == COMMAND_TYPE_SECURITY_FRAME) {
		PUINT_8 pucPkt = (PUINT_8)((struct sk_buff *)prCmd->prPacket)->data;

		prCurCmd->u.rSecFrame.u2EthType =
				(pucPkt[ETH_TYPE_LEN_OFFSET] << 8) | (pucPkt[ETH_TYPE_LEN_OFFSET + 1]);
	} else {
		prCurCmd->u.rCmd.ucCID = prCmd->ucCID;
		prCurCmd->u.rCmd.ucCmdSeqNum = prCmd->ucCmdSeqNum;
		prCurCmd->u.rCmd.fgNeedResp = prCmd->fgNeedResp;
		prCurCmd->u.rCmd.fgSetQuery = prCmd->fgSetQuery;

		prCurCommand->rCmd.ucCID = prCmd->ucCID;
		prCurCommand->rCmd.ucCmdSeqNum = prCmd->ucCmdSeqNum;
		prCurCommand->rCmd.fgNeedResp = prCmd->fgNeedResp;
		prCurCommand->rCmd.fgSetQuery = prCmd->fgSetQuery;

		prCurCommand->u2Counter = u2CurEntryCmd;
		u2CurEntryCmd++;
		if (u2CurEntryCmd == TXED_COMMAND_BUF_MAX_NUM)
			u2CurEntryCmd = 0;
		HAL_MCR_RD(prAdapter, MCR_D2HRM2R, &prCurCommand->u4RelCID);
	}
	u2CurEntry++;
	if (u2CurEntry == TC_RELEASE_TRACE_BUF_MAX_NUM)
		u2CurEntry = 0;
}

VOID wlanTraceReleaseTcRes(P_ADAPTER_T prAdapter, PUINT_8 aucTxRlsCnt, UINT_8 ucAvailable)
{
	static UINT_16 u2CurEntry;
	P_TC_RES_RELEASE_ENTRY prCurBuf = &gprTcReleaseTraceBuffer[u2CurEntry];
	HAL_MCR_RD(prAdapter, MCR_D2HRM2R, &prCurBuf->u4RelCID);
	prCurBuf->u8RelaseTime = sched_clock();
	prCurBuf->ucTc4RelCnt = aucTxRlsCnt[TC4_INDEX];
	prCurBuf->ucAvailableTc4 = ucAvailable;
	u2CurEntry++;
	if (u2CurEntry == TXED_CMD_TRACE_BUF_MAX_NUM)
		u2CurEntry = 0;
}
VOID wlanDumpTxReleaseCount(P_ADAPTER_T prAdapter)
{
	UINT_32 au4WTSR[2];

	HAL_READ_TX_RELEASED_COUNT(prAdapter, au4WTSR);
	DBGLOG(TX, INFO, "WTSR[1]=%d, WTSR[0]=%d\n", au4WTSR[1], au4WTSR[0]);
}

VOID wlanDumpTcResAndTxedCmd(PUINT_8 pucBuf, UINT_32 maxLen)
{
	UINT_16 i = 0;
	P_CMD_TRACE_ENTRY prCmd = gprCmdTraceEntry;
	P_TC_RES_RELEASE_ENTRY prTcRel = gprTcReleaseTraceBuffer;
	if (pucBuf) {
		int bufLen = 0;

		for (; i < TXED_CMD_TRACE_BUF_MAX_NUM/2; i++) {
			bufLen = snprintf(pucBuf, maxLen,
				"%d: Time %llu, Type %d, Content %08x; %d: Time %llu, Type %d, Content %08x\n",
				i*2, prCmd[i*2].u8TxTime, prCmd[i*2].eCmdType, *(PUINT_32)(&prCmd[i*2].u.rCmd.ucCID),
				i*2+1, prCmd[i*2+1].u8TxTime, prCmd[i*2+1].eCmdType,
				*(PUINT_32)(&prCmd[i*2+1].u.rCmd.ucCID));
			if (bufLen <= 0)
				break;
			pucBuf += bufLen;
			maxLen -= bufLen;
		}
		for (i = 0; i < TC_RELEASE_TRACE_BUF_MAX_NUM/2; i++) {
			bufLen = snprintf(pucBuf, maxLen,
				"%d: Time %llu, Tc4Cnt %d, Free %d, CID %08x; %d: Time %llu, Tc4Cnt %d, Free %d CID %08x\n",
				i*2, prTcRel[i*2].u8RelaseTime, prTcRel[i*2].ucTc4RelCnt, prTcRel[i*2].ucAvailableTc4,
				prTcRel[i*2].u4RelCID,
				i*2+1, prTcRel[i*2+1].u8RelaseTime, prTcRel[i*2+1].ucTc4RelCnt,
				prTcRel[i*2+1].ucAvailableTc4, prTcRel[i*2+1].u4RelCID);
			if (bufLen <= 0)
				break;
			pucBuf += bufLen;
			maxLen -= bufLen;
		}
		return;
	}
	for (; i < TXED_CMD_TRACE_BUF_MAX_NUM/4; i++) {
		LOG_FUNC("%d: Time %llu, Type %d, Content %08x; %d: Time %llu, Type %d, Content %08x; ",
			i*4, prCmd[i*4].u8TxTime, prCmd[i*4].eCmdType,
			*(PUINT_32)(&prCmd[i*4].u.rCmd.ucCID),
			i*4+1, prCmd[i*4+1].u8TxTime, prCmd[i*4+1].eCmdType,
			*(PUINT_32)(&prCmd[i*4+1].u.rCmd.ucCID));
		LOG_FUNC("%d: Time %llu, Type %d, Content %08x; %d: Time %llu, Type %d, Content %08x\n",
			i*4+2, prCmd[i*4+2].u8TxTime, prCmd[i*4+2].eCmdType,
			*(PUINT_32)(&prCmd[i*4+2].u.rCmd.ucCID),
			i*4+3, prCmd[i*4+3].u8TxTime, prCmd[i*4+3].eCmdType,
			*(PUINT_32)(&prCmd[i*4+3].u.rCmd.ucCID));
	}
	for (i = 0; i < TC_RELEASE_TRACE_BUF_MAX_NUM/4; i++) {
		LOG_FUNC(
			"%d: Time %llu, Tc4Cnt %d, Free %d, CID %08x; %d: Time %llu, Tc4Cnt %d, Free %d, CID %08x;",
			i*4, prTcRel[i*4].u8RelaseTime, prTcRel[i*4].ucTc4RelCnt,
			prTcRel[i*4].ucAvailableTc4, prTcRel[i*4].u4RelCID,
			i*4+1, prTcRel[i*4+1].u8RelaseTime, prTcRel[i*4+1].ucTc4RelCnt,
			prTcRel[i*4+1].ucAvailableTc4, prTcRel[i*4+1].u4RelCID);
		LOG_FUNC(
			" %d: Time %llu, Tc4Cnt %d, Free %d, CID %08x; %d: Time %llu, Tc4Cnt %d, Free %d, CID %08x\n",
			i*4+2, prTcRel[i*4+2].u8RelaseTime, prTcRel[i*4+2].ucTc4RelCnt,
			prTcRel[i*4+2].ucAvailableTc4, prTcRel[i*4+2].u4RelCID,
			i*4+3, prTcRel[i*4+3].u8RelaseTime, prTcRel[i*4+3].ucTc4RelCnt,
			prTcRel[i*4+3].ucAvailableTc4, prTcRel[i*4+3].u4RelCID);
	}
}
VOID wlanDumpCommandFwStatus(VOID)
{
	UINT_16 i = 0;
	P_COMMAND_ENTRY prCmd = gprCommandEntry;

	LOG_FUNC("Start\n");
	for (; i < TXED_COMMAND_BUF_MAX_NUM; i++) {
		LOG_FUNC("%d: Time %llu, Content %08x, Count %x, RelCID %08x, ReadFwValue %08x, ReadFwTime %llu\n",
			i, prCmd[i].u8TxTime, *(PUINT_32)(&prCmd[i].rCmd.ucCID),
			prCmd[i].u2Counter, prCmd[i].u4RelCID,
			prCmd[i].u4ReadFwValue, prCmd[i].u8ReadFwTime);
	}
}

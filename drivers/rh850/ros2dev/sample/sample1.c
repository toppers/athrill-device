/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.c 739 2017-01-24 10:05:05Z nces-hibino $
 */

#include "Os.h"
#include "t_syslog.h"
#include "t_stdlib.h"
#include "sysmod/serial.h"
#include "sysmod/syslog.h"
#include "sample1.h"

#include "sysmod/banner.h"
#include "target_sysmod.h"
#include "target_serial.h"
#include "target_hw_counter.h"

#include "ros_dev_com.h"
#include "ros_dev.h"
#include "rosdev_gen_types.h"

#define GetHwCnt(x, y)
#define GetAppModeInfo()	(0)

/*
 *  ファイル名，行番号の参照用の変数
 */
extern const char8	*kernel_fatal_file_name;    /* ファイル名 */
extern sint32		kernel_fatal_line_num;      /* 行番号 */

/*
 *  内部関数プロトタイプ宣言
 */
sint32 main(void);
TASK(MainTask);
TASK(HighPriorityTask);
TASK(NonPriTask);
TASK(Task1);
TASK(Task2);
TASK(Task3);
TASK(Task4);
TASK(Task5);
TASK(Task6);
TASK(Task7);
TASK(Task8);
ALARMCALLBACK(SysTimerAlmCb);
static void TaskProk(uint8 task_no);
static uint8 GetCommand(void);
static void PutActTsk(uint8 task_no);
static void PutActNonPriTsk(void);
static void PutTermTsk(uint8 task_no);
static void PutChainTsk(uint8 from_task_no, uint8 to_task_no);
static void PutSchedule(void);
static void PutTaskID(void);
static void PutTaskState(uint8 task_no);
static void PutDisAllInt(void);
static void PutSusAllInt(void);
static void PutSusOSInt(void);
static void PutHwCnt3(void);
static void PutGetCntRes(void);
static void PutGetTskRes(void);
static void PutRelTskRes(void);
static void PutSetEvt(uint8 task_no);
static void PutClrEvt(uint8 task_no);
static void PutGetEvt(uint8 task_no);
static void PutWaitEvt(uint8 task_no);
static void PutArmBase(void);
static void PutArmTick(void);
static void PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
static void PutSetAbs(uint8 alarm_no, uint8 tick_no, uint8 cycle_no);
static void PutCanArm(void);
static void PutAppMode(void);
static void schedule_table_sample_routine(void);

/*
 *  内部データバッファ
 */
static volatile uint8		command_tbl[8];         /* コマンド引渡しテーブル */

/*
 *  内部定数データテーブル
 */
/* 無効イベントマスク値 */
#define invalid_mask	(EventMaskType) (0)

/* イベントマスクテーブル */
static const EventMaskType	event_mask_tbl[] = {
	invalid_mask,
	T2Evt,
	T3Evt,
	invalid_mask,
	invalid_mask
};

/* タスクIDテーブル */
static const TaskType		task_id_tbl[] = {
	Task1,
	Task2,
	Task3,
	Task4,
	Task5
};

/* アラームIDテーブル */
static const AlarmType		alarm_id_tbl[] = {
	ActTskArm,
	SetEvtArm,
	CallBackArm
};

/* ティック値テーブル */
static const TickType		tick_tbl[] = {
	(TickType) 500,
	(TickType) 900
};

/* サイクル値テーブル */
static const TickType		cycle_tbl[] = {
	(TickType) 0,
	(TickType) COUNTER_MIN_CYCLE
};

/* イベントマスク名文字列テーブル */
static const char8			*event_name_tbl[] = {
	"Invalid",
	"T2Evt",
	"T3Evt",
	"Invalid",
	"Invalid"
};

/* タスク名文字列テーブル */
static const char8			*task_name_tbl[] = {
	"Task1",
	"Task2",
	"Task3",
	"Task4",
	"Task5"
};

/* タスク状態文字列テーブル */
static const char8			*task_state_tbl[] = {
	"SUSPENDED",
	"RUNNING",
	"READY",
	"WAITING",
};

/* アラーム名文字列テーブル */
static const char8			*alarm_name_tbl[] = {
	"ActTskArm",
	"SetEvtArm",
	"CallBackArm"
};

/*
 *  APIエラーログマクロ
 *
 *  ErrorHookが有効の場合はErrorHookから
 *  エラーログを出力し, ErrorHookが無効の場合は
 *  以下のマクロよりエラーログ出力を行う
 */
#if defined(CFG_USE_ERRORHOOK)
#define error_log(api)	(api)
#else /* !defined( CFG_USE_ERRORHOOK ) */
#define	error_log(api)										   \
	{														   \
		StatusType ercd;									   \
		ercd = api;     /* 各API実行 */						   \
		if (ercd != E_OK) {									   \
			syslog(LOG_INFO, "Error:%d", atk2_strerror(ercd)); \
		}													   \
	}
#endif /* defined( CFG_USE_ERRORHOOK ) */

/*
 *  ユーザメイン関数
 *
 *  アプリケーションモードの判断と，カーネル起動
 */
sint32
main(void)
{
	AppModeType	crt_app_mode;

	/*
	 *  アプリケーションモードの判断
	 */
	switch (GetAppModeInfo()) {
	case 0:
		crt_app_mode = AppMode1;
		break;
	case 1:
		crt_app_mode = AppMode2;
		break;
	default:
		crt_app_mode = AppMode3;
		break;
	}

	/*
	 *  カーネル起動
	 */
	StartOS(crt_app_mode);

	while (1) {
	}
}   /* main */

/*
 *  メインタスク
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
TASK(MainTask)
{
	RosDevInt32Type pub_data;
	RosDevInt32Type sub_data;
	RosReqType req_pub;
	RosReqType req_sub;

	SetRelAlarm(MainCycArm, TICK_FOR_10MS, TICK_FOR_10MS);

	pub_data.data = 0;
	req_pub.id = 0;
	req_pub.ret = 0;
	req_pub.datalen = sizeof(pub_data);
	req_pub.ptr = (RosDevDataPtrType)&pub_data;

	req_sub.id = 0;
	req_sub.ret = 0;
	req_sub.datalen = sizeof(sub_data);
	req_sub.ptr = (RosDevDataPtrType)&sub_data;

	while (1) {
		WaitEvent(MainEvt);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt);

		rosdev_read_data(&req_sub);
		if (req_sub.ret == 0) {
			pub_data.data = sub_data.data;
			rosdev_write_data(&req_pub);
		}
	}

	syslog(LOG_INFO, "MainTask TERMINATE");
	error_log(TerminateTask());
}   /* TASK( MainTask ) */

/*
 *  最高優先度タスク
 *
 *  各タスクのプリエンプト確認用
 */
TASK(HighPriorityTask)
{
	syslog(LOG_INFO, "HighPriorityTask ACTIVATE");
	error_log(TerminateTask());
}   /* TASK( HighPriorityTask ) */


/*
 *  ノンプリエンプティブタスク
 *
 *  実行中はプリエンプトしないことの確認用
 */
TASK(NonPriTask)
{
	syslog(LOG_INFO, "NonPriTask ACTIVATE");
	syslog(LOG_INFO, "Call ActivateTask(HighPriorityTask)");
	error_log(ActivateTask(HighPriorityTask));
	syslog(LOG_INFO, "NonPriTask TERMINATE");

	error_log(TerminateTask());
}   /* TASK( NonPriTask ) */


/*
 *  並列実行タスク1
 */
TASK(Task1)
{
	TaskProk(0U);
}   /* TASK( Task1 ) */


/*
 *  並列実行タスク2
 */
TASK(Task2)
{
	TaskProk(1U);
}   /* TASK( Task2 ) */


/*
 *  並列実行タスク3
 */
TASK(Task3)
{
	TaskProk(2U);
}   /* TASK( Task3 ) */


/*
 *  並列実行タスク4
 */
TASK(Task4)
{
	TaskProk(3U);
}   /* TASK( Task4 ) */


/*
 *  並列実行タスク5
 */
TASK(Task5)
{
	TaskProk(4U);
}   /* TASK( Task5 ) */


/*
 *  並列実行タスク内部処理
 *
 *  メインタスクから通知されたコマンドごとの処理実行
 */
static void
TaskProk(uint8 task_no)
{
	uint8 command;          /* コマンド退避バッファ */

	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "%s ACTIVATE", task_name_tbl[task_no]);

	/*
	 *  コマンド実行ループ
	 */
	while (1) {

		/*
		 *  コマンド取得
		 */
		while (command_tbl[task_no] == '\0') {
		}
		command = command_tbl[task_no];
		command_tbl[task_no] = 0U;

		/*
		 *  コマンド判定
		 */
		switch (command) {
		case 'A':
			PutTermTsk(task_no);
			break;
		case '!':
		case '"':
		case '#':
		case '$':
		case '%':
			PutChainTsk(task_no, (command - '!'));
			break;
		case 'z':
			PutTaskID();
			break;
		case 'k':
			PutGetTskRes();
			break;
		case 'K':
			PutRelTskRes();
			break;
		case 'i':
			PutGetCntRes();
			break;
		case 'w':
			PutClrEvt(task_no);
			break;
		case 'W':
			PutWaitEvt(task_no);
			break;
		default:
			/* 上記のコマンド以外の場合，処理は行わない */
			break;
		}
}
}   /* TaskProk */

/*
 *  コマンド受信処理
 */
static uint8
GetCommand(void)
{
	uint8 command;          /* コマンド受信バッファ */

	/*
	 *  コマンドを受信するまでループ
	 */
	command = '\0';
	do {
		WaitEvent(MainEvt);     /* 10msウェイト */
		ClearEvent(MainEvt);
		RecvPolSerialChar(&command);    /* 受信バッファポーリング */
		if (command == '\n') {
			command = '\0';
		}
	} while (command == '\0');


	return(command);
}   /* GetCommand */

/*
 *  ActivateTask 実行・ログ出力処理
 */
static void
PutActTsk(uint8 task_no)
{
	syslog(LOG_INFO, "Call ActivateTask(%s)", task_name_tbl[task_no]);

	error_log(ActivateTask(task_id_tbl[task_no]));

}   /* PutActTsk	*/

/*
 *  ActivateTask 実行(NonPriTask)・ログ出力処理
 */
static void
PutActNonPriTsk(void)
{
	syslog(LOG_INFO, "Call ActivateTask(NonPriTask)");

	error_log(ActivateTask(NonPriTask));
}   /* PutActNonPriTsk */

/*
 *  TerminateTask 実行・ログ出力処理
 */
static void
PutTermTsk(uint8 task_no)
{
	StatusType ercd;        /* エラーコード */

	syslog(LOG_INFO, "%s TERMINATE", task_name_tbl[task_no]);

	ercd = TerminateTask();
	ShutdownOS(ercd);
		}

/*
 *  ChainTask 実行・ログ出力処理
 */
static void
PutChainTsk(uint8 from_task_no, uint8 to_task_no)
{
	StatusType ercd;            /* エラーコード */

	syslog(LOG_INFO, "Call ChainTask(%s)", task_name_tbl[to_task_no]);
	syslog(LOG_INFO, "%s TERMINATE", task_name_tbl[from_task_no]);

	ercd = ChainTask(task_id_tbl[to_task_no]);
	if (ercd == E_OS_LIMIT) {
		syslog(LOG_INFO, "Call TerminateTask()");
		syslog(LOG_INFO, "Because of ChainTask E_OS_LIMIT return");
		ercd = TerminateTask();
	}
	ShutdownOS(ercd);
}   /* PutChainTsk */

/*
 *  Schedule 実行・ログ出力処理
 */
static void
PutSchedule(void)
{
	syslog(LOG_INFO, "Call ActivateTask(HighPriorityTask)");

	error_log(ActivateTask(HighPriorityTask));
	syslog(LOG_INFO, "Call Schedule()");

	error_log(Schedule());
	syslog(LOG_INFO, "Retrun Schedule()");
}   /* PutSchedule	*/

/*
 *  GetTaskID 実行・ログ出力処理
 */
static void
PutTaskID(void)
{
	TaskType task_id;           /* タスクID取得バッファ */

	error_log(GetTaskID(&task_id));

	syslog(LOG_INFO, "TaskID:%d", task_id);
}   /* PutTaskID	*/

/*
 *  GetTaskState 実行・ログ出力処理
 */
static void
PutTaskState(uint8 task_no)
{
	TaskStateType state;        /* タスクID取得バッファ */

	error_log(GetTaskState(task_id_tbl[task_no], &state));

	syslog(LOG_INFO, task_name_tbl[task_no]);
	syslog(LOG_INFO, " State:%s", task_state_tbl[state]);
}   /* PutTaskState	*/

/*
 *  DisableAllInterrupts/EnableAllInterrupts 実行・ログ出力処理
 */
static void
PutDisAllInt(void)
{
	syslog(LOG_INFO, "Call DisableAllInterrupts");

	DisableAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call EnableAllInterrupts");

	EnableAllInterrupts();
}   /* PutDisAllInt	*/

/*
 *  SuspendAllInterrupts/ResumeAllInterrupts 実行・ログ出力処理
 */
static void
PutSusAllInt(void)
{
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();
}   /* PutSusAllInt	*/

/*
 *  SuspendOSInterrupts/ResumeOSInterrupts 実行・ログ出力処理
 */
static void
PutSusOSInt(void)
{
	syslog(LOG_INFO, "Call SuspendOSInterrupts");

	SuspendOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendOSInterrupts");

	SuspendOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call SuspendAllInterrupts");

	SuspendAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeAllInterrupts");

	ResumeAllInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeOSInterrupts");

	ResumeOSInterrupts();

	PutHwCnt3();
	syslog(LOG_INFO, "Call ResumeOSInterrupts");

	ResumeOSInterrupts();
}   /* PutSusOSInt */

/*
 *  割込み動作テスト用HWカウンタ値のログ出力処理
 */
static void
PutHwCnt3(void)
{
	uint8	isr1_cnt = 0U;      /* C1ISR カウント値取得バッファ */
	uint8	isr2_cnt = 0U;      /* C2ISR カウント値取得バッファ */
	uint8	cnt;                /* 出力回数カウンタ */

	for (cnt = 0U; cnt < 3U; cnt++) {
		GetHwCnt(&isr1_cnt, &isr2_cnt);
		syslog(LOG_INFO, "C1ISR Cnt:%d, C2ISR Cnt:%d",
			   isr1_cnt, isr2_cnt);
}
}   /* PutHwCnt3 */

/*
 *  GetResource/ReleaseResource 実行(カウンタリソース)・ログ出力処理
 */
static void
PutGetCntRes(void)
{
	syslog(LOG_INFO, "Call GetResource(CntRes)");
	error_log(GetResource(CntRes));

	PutHwCnt3();
	syslog(LOG_INFO, "Call ReleaseResource(CntRes)");

	error_log(ReleaseResource(CntRes));
}   /* PutGetCntRes	*/

/*
 *  GetResource 実行(タスクレベル)・ログ出力処理
 */
static void
PutGetTskRes(void)
{
	syslog(LOG_INFO, "Call GetResource(TskLevelRes)");

	error_log(GetResource(TskLevelRes));
}   /* PutGetTskRes */

/*
 *  ReleaseResource 実行(タスクレベル)・ログ出力処理
 */
static void
PutRelTskRes(void)
{
	syslog(LOG_INFO, "Call ReleaseResource(TskLevelRes)");

	error_log(ReleaseResource(TskLevelRes));
}

/*
 *  SetEvent 実行・ログ出力処理
 */
static void
PutSetEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call SetEvent(%s, %s)",
		   task_name_tbl[task_no], event_name_tbl[task_no]);

	error_log(SetEvent(task_id_tbl[task_no], event_mask_tbl[task_no]));
}   /* PutSetEvt */

/*
 *  ClearEvent 実行・ログ出力処理
 */
static void
PutClrEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call ClearEvent(%s)", event_name_tbl[task_no]);

	error_log(ClearEvent(event_mask_tbl[task_no]));
}   /* PutClrEvt */

/*
 *  GetEvent 実行・ログ出力処理
 */
static void
PutGetEvt(uint8 task_no)
{
	EventMaskType mask;             /* イベントマスク取得バッファ */

	error_log(GetEvent(task_id_tbl[task_no], &mask));

	syslog(LOG_INFO, "%s Event Mask:0x%x", task_name_tbl[task_no], mask);
}   /* PutGetEvt */

/*
 *  WaitEvent 実行・ログ出力処理
 */
static void
PutWaitEvt(uint8 task_no)
{
	syslog(LOG_INFO, "Call WaitEvent(%s)", event_name_tbl[task_no]);

	error_log(WaitEvent(event_mask_tbl[task_no]));
	syslog(LOG_INFO, "Return WaitEvent(%s)", event_name_tbl[task_no]);
}   /* PutWaitEvt */

/*
 *  GetAlarmBase 実行・ログ出力処理
 */
static void
PutArmBase(void)
{
	AlarmBaseType info;             /* アラームベース情報取得バッファ */

	error_log(GetAlarmBase(MainCycArm, &info));

	syslog(LOG_INFO, "MainCycArm Base:");
	syslog(LOG_INFO, "\tMAXALLOWEDVALUE=%d", info.maxallowedvalue);
	syslog(LOG_INFO, "\tTICKSPERBASE=%d", info.ticksperbase);
	syslog(LOG_INFO, "\tMINCYCLE=%d", info.mincycle);
}   /* PutArmBase */

/*
 *  PutArmTick 実行・ログ出力処理
 */
static void
PutArmTick(void)
{
	TickType tick;              /* 残りティック取得バッファ */

	error_log(GetAlarm(MainCycArm, &tick));

	syslog(LOG_INFO, "MainCycArm Tick:%d", tick);
}   /* PutArmTick */

/*
 *  SetRelAlarm 実行・ログ出力処理
 */
static void
PutSetRel(uint8 alarm_no, uint8 tick_no, uint8 cycle_no)
{
	syslog(LOG_INFO, "Call SetRelAlarm(%s, %d, %d)",
		   alarm_name_tbl[alarm_no], tick_tbl[tick_no], cycle_tbl[cycle_no]);

	error_log(SetRelAlarm(alarm_id_tbl[alarm_no],
						  tick_tbl[tick_no], cycle_tbl[cycle_no]));
}   /* PutSetRel	*/

/*
 *  SetAbsAlarm 実行・ログ出力処理
 */
static void
PutSetAbs(uint8 alarm_no, uint8 tick_no, uint8 cycle_no)
{
	syslog(LOG_INFO, "Call SetAbsAlarm(%s, %d, %d)",
		   alarm_name_tbl[alarm_no], tick_tbl[tick_no], cycle_tbl[cycle_no]);

	error_log(SetAbsAlarm(alarm_id_tbl[alarm_no],
						  tick_tbl[tick_no], cycle_tbl[cycle_no]));
}   /* PutSetAbs */

/*
 *  CancelAlarm 実行・ログ出力処理
 */
static void
PutCanArm(void)
{
	syslog(LOG_INFO, "Call CancelAlarm(CallBackArm)");

	error_log(CancelAlarm(CallBackArm));
}   /* PutCanArm */

/*
 *  GetActiveApplicationMode 実行・ログ出力処理
 */
static void
PutAppMode(void)
{
	switch (GetActiveApplicationMode()) {
	case AppMode1:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode1");
		break;
	case AppMode2:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode2");
		break;
	case AppMode3:
		syslog(LOG_INFO, "ActiveApplicationMode:AppMode3");
		break;
	default:
		syslog(LOG_INFO, "ActiveApplicationMode:Non");
		break;
	}
}   /* PutAppMode */


/*
 *  エラーフックルーチン
 */
#ifdef CFG_USE_ERRORHOOK
void
ErrorHook(StatusType Error)
{
	/*
	 *  エラー要因ごとのパラメータログ出力
	 */
	switch (OSErrorGetServiceId()) {
	case OSServiceId_ActivateTask:
		syslog(LOG_INFO, "Error:%s=ActivateTask(%d)", atk2_strerror(Error), OSError_ActivateTask_TaskID());
		break;
	case OSServiceId_TerminateTask:
		syslog(LOG_INFO, "Error:%s=TerminateTask()", atk2_strerror(Error));
		break;
	case OSServiceId_ChainTask:
		syslog(LOG_INFO, "Error:%s=ChainTask(%d)", atk2_strerror(Error), OSError_ChainTask_TaskID());
		break;
	case OSServiceId_Schedule:
		syslog(LOG_INFO, "Error:%s=Schedule()", atk2_strerror(Error));
		break;
	case OSServiceId_GetTaskID:
		syslog(LOG_INFO, "Error:%s=GetTaskID(0x%p)", atk2_strerror(Error), OSError_GetTaskID_TaskID());
		break;
	case OSServiceId_GetTaskState:
		syslog(LOG_INFO, "Error:%s=GetTaskState(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetTaskState_TaskID(), OSError_GetTaskState_State());
		break;
	case OSServiceId_EnableAllInterrupts:
		syslog(LOG_INFO, "Error:%s=EnableAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_DisableAllInterrupts:
		syslog(LOG_INFO, "Error:%s=DisableAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_ResumeAllInterrupts:
		syslog(LOG_INFO, "Error:%s=ResumeAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_SuspendAllInterrupts:
		syslog(LOG_INFO, "Error:%s=SuspendAllInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_ResumeOSInterrupts:
		syslog(LOG_INFO, "Error:%s=ResumeOSInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_SuspendOSInterrupts:
		syslog(LOG_INFO, "Error:%s=SuspendOSInterrupts()", atk2_strerror(Error));
		break;
	case OSServiceId_GetISRID:
		syslog(LOG_INFO, "Error:%s=GetISRID()", atk2_strerror(Error));
		break;
	case OSServiceId_GetResource:
		syslog(LOG_INFO, "Error:%s=GetResource(%d)", atk2_strerror(Error), OSError_GetResource_ResID());
		break;
	case OSServiceId_ReleaseResource:
		syslog(LOG_INFO, "Error:%s=ReleaseResource(%d)", atk2_strerror(Error), OSError_ReleaseResource_ResID());
		break;
	case OSServiceId_SetEvent:
		syslog(LOG_INFO, "Error:%s=SetEvent(%d, 0x%x)", atk2_strerror(Error),
			   OSError_SetEvent_TaskID(), OSError_SetEvent_Mask());
		break;
	case OSServiceId_ClearEvent:
		syslog(LOG_INFO, "Error:%s=ClearEvent(0x%x)", atk2_strerror(Error), OSError_ClearEvent_Mask());
		break;
	case OSServiceId_GetEvent:
		syslog(LOG_INFO, "Error:%s=GetEvent(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetEvent_TaskID(), OSError_GetEvent_Event());
		break;
	case OSServiceId_WaitEvent:
		syslog(LOG_INFO, "Error:%s=WaitEvent(0x%x)", atk2_strerror(Error), OSError_WaitEvent_Mask());
		break;
	case OSServiceId_GetAlarmBase:
		syslog(LOG_INFO, "Error:%s=GetAlarmBase(0x%p)", atk2_strerror(Error), OSError_GetAlarmBase_AlarmID());
		break;
	case OSServiceId_GetAlarm:
		syslog(LOG_INFO, "Error:%s=GetAlarm(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetAlarm_AlarmID(), OSError_GetAlarm_Tick());
		break;
	case OSServiceId_SetRelAlarm:
		syslog(LOG_INFO, "Error:%s=SetRelAlarm(%d, %d, %d)", atk2_strerror(Error),
			   OSError_SetRelAlarm_AlarmID(), OSError_SetRelAlarm_increment(), OSError_SetRelAlarm_cycle());
		break;
	case OSServiceId_SetAbsAlarm:
		syslog(LOG_INFO, "Error:%s=SetAbsAlarm(%d, %d, %d)", atk2_strerror(Error),
			   OSError_SetAbsAlarm_AlarmID(), OSError_SetAbsAlarm_start(), OSError_SetAbsAlarm_cycle());
		break;
	case OSServiceId_CancelAlarm:
		syslog(LOG_INFO, "Error:%s=CancelAlarm(%d)", atk2_strerror(Error), OSError_CancelAlarm_AlarmID());
		break;
	case OSServiceId_StartScheduleTableRel:
		syslog(LOG_INFO, "Error:%s=StartScheduleTableRel(%d, %d)", atk2_strerror(Error),
			   OSError_StartScheduleTableRel_ScheduleTableID(), OSError_StartScheduleTableRel_Offset());
		break;
	case OSServiceId_StartScheduleTableAbs:
		syslog(LOG_INFO, "Error:%s=StartScheduleTableAbs(%d, %d)", atk2_strerror(Error),
			   OSError_StartScheduleTableAbs_ScheduleTableID(), OSError_StartScheduleTableAbs_Start());
		break;
	case OSServiceId_StopScheduleTable:
		syslog(LOG_INFO, "Error:%s=StopScheduleTable(%d)", atk2_strerror(Error), OSError_StopScheduleTable_ScheduleTableID());
		break;
	case OSServiceId_NextScheduleTable:
		syslog(LOG_INFO, "Error:%s=NextScheduleTable(%d, %d)", atk2_strerror(Error),
			   OSError_NextScheduleTable_ScheduleTableID_From(), OSError_NextScheduleTable_ScheduleTableID_To());
		break;
	case OSServiceId_GetScheduleTableStatus:
		syslog(LOG_INFO, "Error:%s=GetScheduleTableStatus(%d, 0x%p)", atk2_strerror(Error),
			   OSError_GetScheduleTableStatus_ScheduleTableID(), OSError_GetScheduleTableStatus_ScheduleStatus());
		break;
	case OSServiceId_GetActiveApplicationMode:
		syslog(LOG_INFO, "Error:%s=GetActiveApplicationMode()", atk2_strerror(Error));
		break;
	case OSServiceId_StartOS:
		syslog(LOG_INFO, "Error:%s=StartOS()", atk2_strerror(Error));
		break;
	case OSServiceId_ShutdownOS:
		syslog(LOG_INFO, "Error:%s=ShutdownOS()", atk2_strerror(Error));
		break;
	case OSServiceId_IncrementCounter:
		syslog(LOG_INFO, "Error:%s=IncrementCounter(%d)", atk2_strerror(Error), OSError_IncrementCounter_CounterID());
		break;
	case OSServiceId_TaskMissingEnd:
		syslog(LOG_INFO, "Error:%s=MissingEnd()", atk2_strerror(Error));
		break;
	default:
		syslog(LOG_INFO, "Error:%s=UnKnownFunc()", atk2_strerror(Error));
		break;
	}

}   /* ErrorHook */
#endif /* CFG_USE_ERRORHOOK */

/*
 *  プレタスクフックルーチン
 *
 *  空ルーチンを呼出す
 */
#ifdef CFG_USE_PRETASKHOOK
void
PreTaskHook(void)
{
}   /* PreTaskHook */
#endif /* CFG_USE_PRETASKHOOK */

/*
 *  ポストタスクフックルーチン
 *
 *  空ルーチンを呼出す
 */
#ifdef CFG_USE_POSTTASKHOOK
void
PostTaskHook(void)
{
}   /* PostTaskHook */
#endif /* CFG_USE_POSTTASKHOOK */

/*
 *  スタートアップフックルーチン
 */
#ifdef CFG_USE_STARTUPHOOK
#ifdef TOPPERS_ENABLE_SYS_TIMER
extern void target_timer_initialize(void);
#endif /* TOPPERS_ENABLE_SYS_TIMER */

void
StartupHook(void)
{
#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_initialize();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	syslog_initialize();
	syslog_msk_log(LOG_UPTO(LOG_INFO));
	InitSerial();
	print_banner();
}   /* StartupHook */
#endif /* CFG_USE_STARTUPHOOK */

/*
 *  シャットダウンフックルーチン
 */
#ifdef CFG_USE_SHUTDOWNHOOK
#ifdef TOPPERS_ENABLE_SYS_TIMER
extern void target_timer_terminate(void);
#endif /* TOPPERS_ENABLE_SYS_TIMER */

void
ShutdownHook(StatusType Error)
{
	/* 終了ログ出力 */
	syslog(LOG_INFO, "");
	syslog(LOG_INFO, "Sample System ShutDown");
	syslog(LOG_INFO, "ShutDownCode:%s", atk2_strerror(Error));
	syslog(LOG_INFO, "");

	if (Error == E_OS_SYS_ASSERT_FATAL) {
		syslog(LOG_INFO, "fatal_file_name:%s", kernel_fatal_file_name);
		syslog(LOG_INFO, "kernel_fatal_line_num:%d", kernel_fatal_line_num);
	}

#ifdef TOPPERS_ENABLE_SYS_TIMER
	target_timer_terminate();
#endif /* TOPPERS_ENABLE_SYS_TIMER */
	TermSerial();

}   /* ShutdownHook */
#endif /* CFG_USE_SHUTDOWNHOOK */

/*
 *  プロテクションフックルーチン
 */
#ifdef CFG_USE_PROTECTIONHOOK
ProtectionReturnType
ProtectionHook(StatusType FatalError)
{
	StatusType ercd;

	syslog(LOG_INFO, "");
	syslog(LOG_INFO, "ProtectionHook");

	if (FatalError == E_OS_STACKFAULT) {
		syslog(LOG_INFO, "E_OS_STACKFAULT");
		ercd = PRO_SHUTDOWN;
	}
	else if (FatalError == E_OS_PROTECTION_EXCEPTION) {
		syslog(LOG_INFO, "E_OS_PROTECTION_EXCEPTION");
		ercd = PRO_IGNORE;
	}
	else {
		ercd = PRO_SHUTDOWN;
	}

	return(ercd);
}
#endif /* CFG_USE_PROTECTIONHOOK */

/*
 *  システムタイマによるアラームコールバック
 */
ALARMCALLBACK(SysTimerAlmCb)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "CallBackArm Expire");

}   /* ALARMCALLBACK(SysTimerAlmCb) */

/*
 *  IncrementCounter確認用アラームコールバック
 */
ALARMCALLBACK(SampleAlmCb)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "SampleArm Expire");

}   /* ALARMCALLBACK( SampleAlmCb ) */

/*
 *  IncrementCounter確認用アラームコールバック
 */
ALARMCALLBACK(SampleAlmCb2)
{
	/*
	 *  コールバック実行ログ出力
	 */
	syslog(LOG_INFO, "SampleArm2 Expire");

}   /* ALARMCALLBACK( SampleAlmCb2 ) */

/*
 *  スケジュールテーブルテスト用メインループ
 *
 *  ユーザコマンドの受信と，コマンドごとの処理実行
 */
static void
schedule_table_sample_routine(void)
{
	uint8					command;                /* コマンドバッファ */
	ScheduleTableType		scheduletable_id;       /* コマンド引数バッファ */
	ScheduleTableStatusType	status;                 /* スケジュール状態引数 */
	TickType				val;                    /* カウンタの現在値 */
	uint8					flag = FALSE;           /* リターンするか判定するためのフラグ */

	syslog(LOG_INFO, "\t[ schedule table sample routine IN, press 't' OUT ]");
	syslog(LOG_INFO, "");

	scheduletable_id = scheduletable1;
	/*
	 *  コマンド実行ループ
	 */
	while (1) {

		WaitEvent(MainEvt);     /* 10msの作業時間待ち */
		ClearEvent(MainEvt);

		/*
		 *  入力コマンド取得
		 */
		syslog(LOG_INFO, "Input Command:");
		command = GetCommand();
		syslog(LOG_INFO, "%c", command);

		/*
		 *  コマンド判定
		 */
		switch (command) {
		case '1':
			scheduletable_id = scheduletable1;
			break;
		case '2':
			scheduletable_id = scheduletable2;
			break;
		case 'i':
			IncrementCounter(SchtblSampleCnt);
			val = 0U;
			GetCounterValue(SchtblSampleCnt, &val);
			if ((val % 5U) == 0U) {
				syslog(LOG_INFO, "\tGetCounterValue(SchtblSampleCnt ) = %d", val);
			}
			break;
		case 's':
			syslog(LOG_INFO, "\tStartScheduleTableRel(scheduletable%d, 5)", scheduletable_id + 1U);
			error_log(StartScheduleTableRel(scheduletable_id, 5U));
			break;
		case 'S':
			syslog(LOG_INFO, "\tStartScheduleTableAbs(scheduletable%d, 5)", scheduletable_id + 1U);
			error_log(StartScheduleTableAbs(scheduletable_id, 5U));
			break;
		case 'f':
			syslog(LOG_INFO, "\tStopScheduleTable(scheduletable%d)", scheduletable_id + 1U);
			error_log(StopScheduleTable(scheduletable_id));
			break;
		case 'n':
			syslog(LOG_INFO, "\tNextScheduleTable(scheduletable%d, scheduletable%d)", scheduletable_id + 1U, scheduletable2 + 1U);
			error_log(NextScheduleTable(scheduletable_id, scheduletable2));
			break;
		case 'N':
			syslog(LOG_INFO, "\tNextScheduleTable(scheduletable%d, scheduletable%d)", scheduletable_id + 1U, scheduletable1 + 1U);
			error_log(NextScheduleTable(scheduletable_id, scheduletable1));
			break;
		case 'g':
			status = 0U;
			syslog(LOG_INFO, "\tGetScheduleTableStatus(scheduletable%d, status)", scheduletable_id + 1U);
			error_log(GetScheduleTableStatus(scheduletable_id, &status));
			syslog(LOG_INFO, "\tstatus = %d", status);
			break;
		case 't':
			syslog(LOG_INFO, "\t[ schedule table sample routine OUT, press 't' IN ]");
			flag = TRUE;
			break;
		case 'q':
			ShutdownOS(E_OK);
			break;
		case 'Q':
			ShutdownOS(E_OS_STATE);
			break;
		default:
			/* コマンドが上記のケース以外なら処理は行わない */
			break;
		}
		/*  フラグが立っていた場合，リターンする  */
		if (flag == TRUE) {
			return;
		}
	}
}   /* schedule_table_sample_routine */

/*
 *  スケジュールテーブル確認用タスク6
 */
TASK(Task6)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task6 ACTIVATE");
	WaitEvent(T6Evt);
	syslog(LOG_INFO, "Task6 FINISH");
	TerminateTask();
}   /* TASK( Task6 ) */


/*
 *  スケジュールテーブル確認用タスク7
 */
TASK(Task7)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task7 ACTIVATE");
	WaitEvent(T7Evt);
	syslog(LOG_INFO, "Task7 FINISH");
	TerminateTask();
}   /* TASK( Task7 ) */


/*
 *  スケジュールテーブル確認用タスク8
 */
TASK(Task8)
{
	/*
	 *  タスク起動ログ出力
	 */
	syslog(LOG_INFO, "Task8 ACTIVATE");
	WaitEvent(T8Evt);
	syslog(LOG_INFO, "Task8 FINISH");
	TerminateTask();
}   /* TASK( Task8 ) */

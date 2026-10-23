/*
类型    说明
================
1       上电事件
2       IO事件
3       模块丢失报警
4       参数设置事件（修改参数标识，新设置值）
5       充电事件（开始，结束，异常结束）
6       温度不均衡
7       温度过高
8       温度过低
9       极柱温度不均衡
10      极柱温度过高
11      单体过压
12      单体欠压
13      单体电压不均衡
14      总电压过压
15      总电压欠压
16      充电过流
17      回馈过流
18      放电过流
19      SOC高
20      SOC低
21      绝缘报警
22      充电电压大
23      清除报警记录
24      清除历史数据
25      极注温度过低
26      铁电读错误
27      铁电写错误
28      FLASH读错误
29      FLASH写错误
30      时钟错误
31      参数丢失
32      时间丢失
33      与采集板通讯错误
34      电流ADC错误
35      绝缘ADC错误
36      绝缘失效
37      加热事件
39      温度失效
*/

// 事件结构体: (最大 16 Bytes)
typedef struct
{
    time_t      time;
    UINT8       type;
    UINT8       rep;    // 重复次数

    union
    {
    struct        // 上电事件
    {
        time_t  pwdn;   // 停电时刻
    } pwr;

    struct           // IO事件
    {
        UINT32  serial_num;
        UINT16  in;     // 输入端口状态
        UINT16  out;    // 输出端口状态
    } io;

    struct         // 模块丢失报警
    {
        UINT8   id;     // 丢失的模块
        UINT8   actives;// 当前模块数量
        UINT8   lecunum;// 参数配置模块数量
    } lost;

    struct      // 参数设置事件
    {
        UINT16  old;
        UINT16  value;
        UINT8   which;  // 0-其它参数，1-温度超高，2-温度过高，...
        UINT8   way;    // 0-HMI，1-PC，2-仪表, 3-其他
    } para;
    
    struct      // 充电事件
    {
        UINT16  volt;
        UINT16  maxv;
        UINT8   maxt;
        UINT8   type;   // 0-停止充电，1-开始充电，4-异常终止
    } cha;
    
    struct      // 极注、单体温度不均衡
    {
        UINT8   level;
        UINT8   err;    // 温差
    } tmpunba;
    
    struct      // 温度过高、低
    {
        UINT32  serial_num;//
        UINT8   level;
        UINT8   batt;   // 温度传感器号
        UINT8   tmp;
    } tmp;
    
    struct     // 单体过、欠压
    {
        UINT32  serial_num;
        UINT8   level;
        UINT8   batt;
        UINT16  volt;
    } vol;
    
    struct      // 单体电压不均衡
    {
        UINT16  err;
        UINT8   level;
    } volunba;
    
    struct      // 总电压过、欠压
    {
        UINT16  volt;
        UINT8   level;
    } tovol;
    
    struct       // 回馈、充电、放电过流
    {
        UINT16  volt;
        UINT16  current;
        UINT8   level;
    } cur;
    
    struct      //SOC高、低
    {
        UINT16  volt;
        UINT8   level;
        UINT8   soc;
    } soc;
    
    struct      // 绝缘故障
    {
        UINT16  volt;
        UINT16  res;
        UINT8   level;
    } iso;
    
    struct
    {
        UINT8   state;    //加热开关标志
        UINT8   id;
        UINT8   min_t;    //最低温度
        UINT8   max_t;    //最高温度
        UINT16  min_v;    //最小电压
        UINT8   reason;   //操作原因
    } warm;
    
    struct 
    {
        UINT8 maxt;
        UINT8 mint;
        UINT8 avgt;
    } temp;
    } u;

    UINT8   crc;
} DBEvent;


/*****************************************************************************/
/* 事件报警生成函数                                                          */
/*****************************************************************************/
void EV_Make (DBEvent *ev);
void EV_Make_Charger (UINT8 type);
void EV_Make_ParaChange (UINT8 way, UINT8 which, UINT16 old, UINT16 val);

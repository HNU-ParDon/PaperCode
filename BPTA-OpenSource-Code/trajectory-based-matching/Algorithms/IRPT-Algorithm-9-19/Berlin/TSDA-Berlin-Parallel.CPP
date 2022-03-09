//input:T,W,PT, PW
//output:stable matching
// Task preference priority Greedy -算法3
//5-13.9:42 任务向工人提出匹配请求，先判断任务是否在工人的偏好列表里，重复定义PT了
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h> //worker
#include <map>
#include <regex>
#include <string>
#include <cmath>
#include <random>
#include <time.h>
#include <Windows.h>
using namespace std;

struct POI
{ //POI点的结构
  double X;
  double Y;
};

struct WORKER
{ //worker的结构

  vector<POI> trajectory; //轨迹,容器tra里面顺序存储了POI点信息,轨迹文件单独读取
  double endTime;
  double range;
  double score;
};

struct TASK
{ //task的结构
  double X;
  double Y;
  double Reward;
  double Minscore;
};

struct Worker_Task_Utility
{
  int workerid;
  int taskid;
  double utility;
};

const double c = 1.0 / 1000; //单位距离的工人成本
const double speed = 1000;     //工人的移动速度1000/min,60km/h
const int Number_Task = 500;
const int Number_Worker = 120;
const int Number_BusStop = 4346;
const int Capacity = 5;

#define PI 3.1415926535897932384626433832795
#define EARTH_RADIUS 6378.137    //地球半径 KM
#define Score_Min 60
#define Score_Max 100
#define Reward_Min 1
#define Reward_Max 10

WORKER worker[Number_Worker];                       //Worker数组；从数据集文件中获得
TASK task[Number_Task];                             //Task数组；从数据集文件中获得
vector<pair<int, double>> PT[Number_Task];          //task的偏序列表
vector<pair<int, double>> PW[Number_Worker];        //worker的偏序列表
static double detour_distance[Number_Task][Number_Worker]; //存储detour_distance

int CW_Task[Number_Task];                    //记录number_task个task的当前对象
vector<int> CT_Worker[Number_Worker];             ////记录number_worker个worker的当前对象
int num_of_chased_worker[Number_Task] = {0}; //任务追求过的工人的数量
vector<int> ActiveTask;                      //记录当前活动任务集
double AD[Number_Worker];                    //AD记录worker的剩余可用偏移距离

//stable matching的声明函数
void Initialize(double (&AD)[Number_Worker], vector<int> &ActiveTask, vector<pair<int, double>> (&PT)[Number_Task], double (&Sumdis)[Number_Worker]);
void ShowMatching();
bool AcceptTask(int taskid, int workerid, vector<pair<int, double>> (&PW)[Number_Worker]);
void Update_AD1(int workerid, int taskid, double (&AD)[Number_Worker]);
int  Find_ReplaceTask(int worekrid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]);
void RemoveReplaceTask(int workerid, int taskid, const vector<int> &MinReplaceTask); //从工人已匹配中移除MinReplaceTask
void AddTask_into_AVT(int workerid, int taskid, const vector<int> &MinReplaceTask);  //将 MinReplaceTask加入AT
void Update_AD2(int workerid, int taskid, int MinReplaceTask);                       //更新可用时间和可用偏移距离
int GetIndex_PW(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]);
bool cmp(pair<int, double> a, pair<int, double> b);
vector<pair<int, double>>::iterator IFTaskExist(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]);
double Caculate_Task_Satisfaction(int (&CW_Task)[Number_Task],vector<pair<int, double>> (&PT)[Number_Task]);
double Caculate_Worker_Satisfaction(vector<int> CT_Worker[Number_Worker],vector<pair<int, double>> (&PW)[Number_Worker]);
int GetIndex_PT(int workerid, int taskid, vector<pair<int, double>> (&PT)[Number_Task]);
void ShowDetour_Distance(double (&detour_distance)[Number_Task][Number_Worker],int (&poi)[Number_Task][Number_Worker]);


//读取数据文件的声明函数
void ReadLocationForTask(TASK (&task)[Number_Task]);
void produce_Task_Reward_Minscore(TASK (&task)[Number_Task]); //生成task的其它属性
void showTask(TASK (&task)[Number_Task]);
void Get_Trajectory_locations(WORKER (&worker)[Number_Worker]); //生成轨迹位置文件
void Caculate_Sumdist_Trajectory(double (&Sumdis)[Number_Worker], WORKER (&worker)[Number_Worker]);
void produce_Worker_endTime_range_score(WORKER (&worker)[Number_Worker], double (&Sumdis)[Number_Worker], double endtimeX, double rangeX, double scoreX); //生成worker的其它属性
void showWorker(WORKER (&worker)[Number_Worker]);
double rad(double d);
double GetDistance(double lat1, double lng1, double lat2, double lng2);

//try-task,try-worker
double Caculate_mindist(int workerid, int taskid, int (&poi)[Number_Task][Number_Worker]); //声明函数
void Compute_PTPW(int taskid,vector<pair<int, double>> (&PT)[Number_Task], vector<pair<int, double>> (&PW)[Number_Worker], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker]);

int main()
{
  //for(double TestEndtimeX = 0.1;  TestEndtimeX < 1; TestEndtimeX= TestEndtimeX + 0.1)
//{
  double Sumdis[Number_Worker] = {0};
 static int poi[Number_Task][Number_Worker];  //任务到工人最近的轨迹点
  for (int i = 0; i < Number_Task; i++) //任务的当前工人为空,-1
    CW_Task[i] = -1;

  ReadLocationForTask(task);
  produce_Task_Reward_Minscore(task);
  //showTask(task);
  Get_Trajectory_locations(worker);
  Caculate_Sumdist_Trajectory(Sumdis, worker);

  produce_Worker_endTime_range_score(worker, Sumdis, 0.3, 2000, 1);
  //showWorker(worker);

  double run_time;
  LARGE_INTEGER time_start; //开始时间
  LARGE_INTEGER time_over;  //结束时间
  double dqFreq;            //计时器频率
  LARGE_INTEGER f;          //计时器频率
  QueryPerformanceFrequency(&f);
  dqFreq = (double)f.QuadPart;
  QueryPerformanceCounter(&time_start); //计时开始 

  #pragma omp parallel for num_threads(4) 
  for (int i = 0; i < Number_Task; i++)
  {
    Compute_PTPW(i, PT, PW, detour_distance, poi); //计算PT,PW , 多线程访问PW
    sort(PT[i].begin(), PT[i].end(), cmp);                //对任务的偏序列表排序
  }

/*
    int sum=0;
  for(int i=0; i<Number_Task ;i++)
  {
  if(PT[i].size() == 0)
  {
   //  cout<<i<<"任务没有偏好列表!!!!!!"<<endl;
     sum++;
  }
  else{
        cout<<"任务"<<i<<"的偏好列表:"<<endl;
     for(int j=0; j<PT[i].size();j++){
         cout<<"工人id："<<PT[i][j].first<<",偏好值："<<PT[i][j].second<<endl;
          cout<<endl;
     }
     
     }
     
  }
     cout<<"没有偏好列表的任务数:"<<sum<<endl;
     cout<<endl;
     */
  
  for (int i = 0; i < Number_Worker; i++)
  {
    sort(PW[i].begin(), PW[i].end(), cmp);              //对工人的偏序列表排序
  }
  QueryPerformanceCounter(&time_over);                      //计时结束
  run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq; //乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
  printf("\nSumArrRow run_time：%fs\n", run_time);

/*
    int sum1 = 0 ; 
  for (int i = 0; i <Number_Worker; i++)
  {
    if (PW[i].size() == 0)
    {
    //  cout << i << "工人的偏好列表为空！！！！" << endl;
      sum1++;
    }
    else
   {
     
     cout<<"工人分数："<< worker[i].score<<endl;
     cout<<"工人"<<i<<"的偏好列表共有任务数:"<<PW[i].size()<<endl;
     for(int j=0; j< PW[i].size();j++)
       {  
         cout<<"任务id："<<PW[i][j].first<<"任务报酬："<<task[PW[i][j].first].Reward<<"任务绕路："<<detour_distance[PW[i][j].first][i]<<"任务分数约束："<<task[PW[i][j].first].Minscore<<",偏好值："<<PW[i][j].second<<endl;
         cout<<endl;
       }
       

   }
  }
  cout << "偏好列表为空的工人总数为：" << sum1 << endl;
  cout<< endl;
*/
 //ShowDetour_Distance(detour_distance,poi);
  Initialize(AD, ActiveTask, PT, Sumdis); //初始化ActiveTask任务集和AT【Number_worker】,AD[number_worker]；
  vector<int> NextActiveTask(ActiveTask);

  int matchingTimes = 0;
  while (!ActiveTask.empty())
  {
    //cout << "第" << ++matchingTimes << "轮匹配开始！！！！！" << endl;
    NextActiveTask.clear();
    for (int i = 0; i < ActiveTask.size(); i++) //为ActiveTask重新赋值为NextACTIVETASK
      NextActiveTask.push_back(ActiveTask[i]);

    int matchingnumber = 0, replacematching = 0;
    for (int i = 0; i < ActiveTask.size(); i++)
    {
      int taskid = ActiveTask[i];
      int order = num_of_chased_worker[taskid];
      int worker_to_chase = PT[taskid][order].first;
      //  cout << taskid << "目前已经追求的工人数量:" << order << endl;
      //  cout << "任务的偏好工人数：" << PT[taskid].size() << endl;
      num_of_chased_worker[taskid] = order + 1; //无论匹配成功与否，该任务追过的工人数量加1。

      if (IFTaskExist(worker_to_chase, taskid, PW) != PW[worker_to_chase].end())
      {
        if (AcceptTask(taskid, worker_to_chase, PW))
        {
          CW_Task[taskid] = worker_to_chase;
        
          CT_Worker[worker_to_chase].push_back(taskid);
          //  cout << "工人：" << worker_to_chase << "目前已匹配的任务数：" << CT_Worker[worker_to_chase].size() << endl;
          NextActiveTask.erase(find(NextActiveTask.begin(), NextActiveTask.end(), taskid)); //更新Activetask

          Update_AD1(worker_to_chase, taskid, AD); //更新可用偏移距离
                                                   //      cout << worker_to_chase << "工人的可用时间：" << AT[worker_to_chase] << endl;
                                                   //     cout << "工人-任务匹配成功！！！" << endl;
                                                   //     cout << " 配对为:"
                                                   //          << "(" << taskid << "," << worker_to_chase << ")" << endl;
          matchingnumber++;
          int taskindex = GetIndex_PW(worker_to_chase, taskid, PW);
         
        }
        else
        {
          //     for (int i = 0; i < After_Task.size(); i++)
          //      cout << "aftertask:" << After_Task[i].first << endl;
          //      for (int i = 0; i < RPTask.size(); i++)
          //      cout << "replacetask:" << RPTask[i] << endl;
          //      cout << "endddd!" << endl;
          int MinReplaceTask =  Find_ReplaceTask(worker_to_chase, taskid, PW); //找到报酬最小的任务
          if ( MinReplaceTask != -1)         //存在可替换的任务
          {
            CW_Task[taskid] = worker_to_chase;
              CW_Task[MinReplaceTask] = -1;
            CT_Worker[worker_to_chase].push_back(taskid);
       // RemoveReplaceTask(worker_to_chase, taskid, MinReplaceTask);  //从工人已匹配中移除MinReplaceTask
            CT_Worker[worker_to_chase].erase(find(CT_Worker[worker_to_chase].begin(), CT_Worker[worker_to_chase].end(), MinReplaceTask));
      // AddTask_into_AT(worker_to_chase, taskid, MinReplaceTask);   //将 MinReplaceTask加入AT
            if (num_of_chased_worker[MinReplaceTask] < PT[MinReplaceTask].size()) //****4.29：21：13 Active任务需要还有可求婚的对象
                 NextActiveTask.push_back(MinReplaceTask);
            NextActiveTask.erase(find(NextActiveTask.begin(), NextActiveTask.end(), taskid)); //将taskid任务移除AT；

            Update_AD2(worker_to_chase, taskid, MinReplaceTask); //更新可用偏移距离
           //  cout << "替换成功!" << endl;
            replacematching++;
         //  int taskindex = GetIndex_PW(worker_to_chase, taskid, PW);
         //   int MinRTindex = GetIndex_PW(worker_to_chase, MinReplaceTask, PW);
         //   int workerindex = GetIndex_PT(worker_to_chase, MinReplaceTask, PT);
          }
          else
          {
            //      cout << "无可替换任务！！！" << endl; //该任务追过的工人数量加1
            //      cout << "llll" << endl;
          }
        }
      }
      if (num_of_chased_worker[taskid] == PT[taskid].size()) //匹配到偏好列表的最后一个，则从ActivetASK中移除！
      {
        vector<int>::iterator iter1 = find(NextActiveTask.begin(), NextActiveTask.end(), taskid);
        if (iter1 != NextActiveTask.end())
        {
          NextActiveTask.erase(find(NextActiveTask.begin(), NextActiveTask.end(), taskid));
        }
        vector<int>::iterator iter2 = find(NextActiveTask.begin(), NextActiveTask.end(), taskid);
        if (iter2 == NextActiveTask.end())
        {
          //    cout << "任务偏好列表已达最后一个，任务已移除！" << endl;
          //    cout << endl;
        }
        
      }
       
    }
    //   cout << "第" << matchingTimes << "轮匹配结束！！！！！" << endl;
    //   cout << "本轮匹配到配对数量：" << matchingnumber << "\t"
    //       << "本轮替换的次数:" << replacematching << endl;

    ActiveTask.clear();
    for (int i = 0; i < NextActiveTask.size(); i++) //为ActiveTask重新赋值为NextACTIVETASK
      ActiveTask.push_back(NextActiveTask[i]);
    //   cout << "下一轮匹配的任务数：" << NextActiveTask.size() << endl;
    //   cout << "end" << endl;
  }


ShowMatching(); //输出配对
  //cout<<"testendtime:"<<TestEndtimeX<<endl;
cout<<"任务的平均满意度："<<Caculate_Task_Satisfaction(CW_Task,PT)<<endl;
cout<<"工人的平均满意度："<<Caculate_Worker_Satisfaction(CT_Worker,PW)<<endl;
//}
  return 0;
}

void ShowMatching()
{ //输出匹配
  int count = 0;
  cout << "匹配情况如下：" << endl;
  for (int i = 0; i < Number_Task; i++)
  {
 //   cout << "（" << i << "," << CW_Task[i] << ")" << endl;
    if (CW_Task[i] != -1)
      count++;
  }
int count2 =0;
    for (int i = 0; i < Number_Worker; i++)
  {
 //   cout << "（" << i << "," << CW_Task[i] << ")" << endl;
    if (CT_Worker[i].size() != 0)
    for(int j=0; j<CT_Worker[i].size();j++)
           count2++;
  } 


  map<int,int> worker_count;
  for(int i=0;i<Number_Task;i++){
   int workerid = CW_Task[i] ;
   if(workerid != -1)
      worker_count[workerid]++;
}  
   cout << "匹配对总数1为:" << count << endl;
   cout << "匹配对总数2为:" << count2 << endl;
   cout << "匹配工人总数为:" << worker_count.size() << endl;
}

void Initialize(double (&AD)[Number_Worker], vector<int> &ActiveTask, vector<pair<int, double>> (&PT)[Number_Task], double (&Sumdis)[Number_Worker])
{
  //初始化ActiveTask,AD
  for (int i = 0; i < Number_Task; i++)
  {
    if (PT[i].size() != 0) //偏好列表不为0才是可用任务
    {                      //   cout<<i<<"任务YOU偏好列表"<<endl;
                           //   cout<<PT[i].size()<<endl;
      ActiveTask.push_back(i);
    }
    // else
    //   cout<<i<<"任务无偏好列表"<<endl;
  }
  for (int j = 0; j < Number_Worker; j++)
  {
    AD[j] = worker[j].endTime * speed - Sumdis[j];
  }
}

bool AcceptTask(int taskid, int workerid, vector<pair<int, double>> (&PW)[Number_Worker])
{ //判断当前任务是否可和工人配对
  //  cout << "taskid:" << taskid << "workerid:" << workerid << endl;
  double dd = 2 * detour_distance[taskid][workerid];
  //  cout << "绕路距离：" << dd << endl;
  //  cout << "AD:" << AD[workerid] << "AT:" << AT[workerid] << endl;
 int assignedtask = CT_Worker[workerid].size();
// cout<<"workerid"<<workerid<<","<<assignedtask<<endl;
 if(assignedtask < Capacity){    //工人容量未达上限且可用绕路距离够，任务和工人才可以匹配
     if ((dd <= AD[workerid]))
        {
        ///    cout << "工人" << workerid << "接受任务!" << taskid << endl;
       return true;
       }
       else
           return false;
 }
  else
  {
    //    cout << "可用时间或绕路距离不够，工人" << workerid << "暂时不接受任务!" << endl;
    //   cout << "工人目前的配对情况：" << endl;
    //   for (int i = 0; i < CT_Worker[workerid].size(); i++)
    //     cout << CT_Worker[workerid][i] << endl;

    /*     cout<<"工人的偏好列表："<<endl;
        if(PW[workerid].size() >0) 
        for(int i=0 ;i<PW[workerid].size();i++)
        cout<<PW[workerid][i].first<<endl;
        else
        cout<<"工人偏好列表为空！"<<endl;
        cout<<endl;
        */
    return false;
  }
}

void Update_AD1(int workerid, int taskid, double (&AD)[Number_Worker])
{ //加入任务后，更新工人的可用偏移距离
  double dd = 2 * detour_distance[taskid][workerid];
  AD[workerid] = AD[workerid] - dd;
}

int Find_ReplaceTask(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]) //4-22 9:37修改该算法
{                                                                                                                  //返回工人的已匹配列表中排在当前任务taskid后面的任务集
   int reTaskindex = -1;
   int replacetaskid = -1;
   double minProfit = __DBL_MAX__;   //极大值
  int t1 = GetIndex_PW(workerid, taskid, PW);
  for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
  {
    int t2 = GetIndex_PW(workerid, *it, PW);
    if (t1 < t2) //*it 排在 taskid的后面
    {
      if( AD[workerid] + 2 * detour_distance[workerid][*it] > 2 * detour_distance[workerid][taskid] );
         if( t2 > reTaskindex ) 
           {
             reTaskindex  = t2 ;
             replacetaskid = *it ;
           }
    }
  }
  return replacetaskid;
}

 

vector<vector<int>> FindReplace_Task(int workerzid, int taskid, const vector<pair<int, int>> &After_Task)
{ //返回多个可替换任务集,线性规划
  vector<vector<int>> RT;
  /*   vector<int> rt;
    int size=After_Task.size();
    vector<vector<int>> aa;
    for(int i=0; i<size; i++)
        for(int j=0;j < 2; j++)
        aa.at(i).at(j)=j;
    
    for(int a=0; a<aa.size(); a++)
       for(int b=0; b< aa.size()-1; b++ )
*/
  return RT;
}



void RemoveReplaceTask(int workerid, int taskid, const vector<int> &MinReplaceTask)
{
  //从工人已匹配中移除MinReplaceTask
  for (int i = 0; i < MinReplaceTask.size(); i++)
  {
    vector<int>::iterator iter = find(CT_Worker[workerid].begin(), CT_Worker[workerid].end(), MinReplaceTask.at(i));
    if (iter != CT_Worker[workerid].end())
    {
      CT_Worker[workerid].erase(iter);
      break;
    }
  }
}

void AddTask_into_AVT(int workerid, int taskid, const vector<int> &MinReplaceTask)
{
  //将 MinReplaceTask加入ActiveTask
  for (int i = 0; i < MinReplaceTask.size(); i++)
  {
    ActiveTask.push_back(MinReplaceTask.at(i));
  }
}

void Update_AD2(int workerid, int taskid, int MinReplaceTask)
{
  //更新替换后的可用绕路距离
  double dd = 2 * detour_distance[taskid][workerid];         //加入的任务
  double DD = 2 * detour_distance[MinReplaceTask][workerid]; //移除的任务
  AD[workerid] = AD[workerid] - dd + DD;
}

int GetIndex_PW(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker])
{
  //获取任务在PW中的排序位置
  int i = 0;
  for (i; i < PW[workerid].size(); i++)
  {
    if (PW[workerid][i].first == taskid)
      break;
  }
  return i;
}

bool cmp(pair<int, double> a, pair<int, double> b) //比较函数
{
  return a.second > b.second;
}

void ReadLocationForTask(TASK (&task)[Number_Task])
{
  //读取任务的位置信息
  ifstream in("F:\\CPP\\dataset\\Berlin\\Task_LocationBER.txt"); //打开文件
  //ifstream in2("F:\\CPP\\dataset\\RestaurantsAMS_Reward_Minscore.txt"); //打开文件
  //读数据。
  for (int i = 0; i < Number_Task; ++i)
  {
    for (int j = 0; j < 5; ++j)
    {
      double temp;
      in >> temp;
      if (j == 1)
        task[i].X = temp;
      if (j == 2)
        task[i].Y = temp;
    }
  }
  in.close(); //关闭文件
}

void Get_Trajectory_locations(WORKER (&worker)[Number_Worker])
{
  //读取工人路径的轨迹点的位置
	ifstream in1("F:\\CPP\\dataset\\Berlin\\BusRoutesAsStopsBER.txt"); 
	ifstream in2("F:\\CPP\\dataset\\\\Berlin\\BusStopsBER.txt"); 
  struct BusStop
  {
    int BusStopId;
    double X;
    double Y;
    /* data */
  };
  vector<BusStop> BusStops;
  for (int i = 0; i < Number_BusStop; ++i)
  {
    for (int j = 0; j < 5; ++j)
    {
      BusStop temp;
      double tm;
      in2 >> tm;
      if (j == 0)
        temp.BusStopId = (int)tm;

      if (j == 1)
        temp.X = tm;

      if (j == 2)
      {
        temp.Y = tm;
        BusStops.push_back(temp);
      }
    }
  }
  in2.close(); //关闭文件

  vector<POI> trajectory;
  string line;
  regex pat_regex("[[:digit:]]+");
  int i = 0, p=0;
  while (p < Number_Worker)
  { 
    getline(in1, line);   //按行读取
    int j = 0;
    for (sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it)
    {
      if (j != 0)
      {
        int temp = stoi(it->str());
        vector<BusStop>::iterator itt;
        for (itt = BusStops.begin(); itt != BusStops.end(); itt++)
        {
          int id = (*itt).BusStopId;
          if (temp == id)
          {
            POI poi;
            poi.X = (*itt).X;
            poi.Y = (*itt).Y;
            worker[i].trajectory.push_back(poi);
            break;
          }
        }
      }
      j++;
    }

    i++;
    p++;
  }
  in1.close();
}

void Caculate_Sumdist_Trajectory(double (&Sumdis)[Number_Worker], WORKER (&worker)[Number_Worker])
{
  //计算轨迹的总路径长度
  for (int i = 0; i < Number_Worker; i++)
  {
    double sum = 0;
    for (int j = 0; j < worker[i].trajectory.size() - 1; j++)
    {
      sum = sum + GetDistance(worker[i].trajectory[j].Y, worker[i].trajectory[j].X, worker[i].trajectory[j + 1].Y, worker[i].trajectory[j + 1].X);
      //	  cout<<sum<<endl;
    }
    Sumdis[i] = sum;
    //   cout<<i<<" "<<"all:" << Sumdis[i] <<endl;
  }

/*int tongji[6]={0};

for(int i=0;i<Number_Worker;i++)
{

 if(Sumdis[i] < 3000)
 tongji[0]++;
 else  if(Sumdis[i] < 6000)
 tongji[1]++;
 else  if(Sumdis[i] < 9000)
 tongji[2]++;
 else  if(Sumdis[i] < 12000)
 tongji[3]++;
 else  if(Sumdis[i] < 15000)
 tongji[4]++;
 else
  tongji[5]++;
}

cout<<"旅行距离小于3000m的工人数有："<<tongji[0]<<endl;
cout<<"旅行距离小于6000m的工人数有："<<tongji[1]<<endl;
cout<<"旅行距离小于9000m的工人数有："<<tongji[2]<<endl;
cout<<"旅行距离小于12000m的工人数有："<<tongji[3]<<endl;
cout<<"旅行距离小于15000m的工人数有："<<tongji[4]<<endl;
cout<<"旅行距离大于15000m的工人数有："<<tongji[5]<<endl;
cout<<endl;
*/

}

void produce_Task_Reward_Minscore(TASK (&task)[Number_Task])
{
  //随机生成task的reward和minscore
  default_random_engine e1, e2;
  uniform_real_distribution<double> u1(Score_Min, Score_Max);    //score
  uniform_int_distribution<unsigned> u2(Reward_Min, Reward_Max); //reward
  for (int i = 0; i < Number_Task; ++i)
  {
    task[i].Minscore = u1(e1);
    task[i].Reward = u2(e2);
  }
}

void produce_Worker_endTime_range_score(WORKER (&worker)[Number_Worker], double (&Sumdis)[Number_Worker], double endtimeX, double rangeX, double scoreX)
{
  //设置参数endtime,range,score;
  default_random_engine e;
  uniform_real_distribution<double> u(Score_Min, Score_Max); //score
  for (int i = 0; i < Number_Worker; ++i)
  {
    worker[i].endTime = (Sumdis[i] / speed)  * (1 + endtimeX); //每个工人的截止时间设置为其正常到达时间多endtimeX倍,单位：Min
    worker[i].range = rangeX;
    worker[i].score = u(e) * scoreX;
  }
}

void showTask(TASK (&task)[Number_Task])
{
  for (int i = 0; i < Number_Task; i++)
  {
    cout << "id: " << i << " ";
    cout << "X：" << task[i].X << " ";
    cout << "Y：" << task[i].Y << " ";
    cout << "minscore：" << task[i].Minscore << " ";
    cout << "reward：" << task[i].Reward << " ";
    cout << endl;
  }
}

void showWorker(WORKER (&worker)[Number_Worker])
{

  for (int i = 0; i < Number_Worker; i++)
  {
    cout << "工人id: " << i << endl;
    for (int j = 0; j < worker[i].trajectory.size(); j++)
    {
      cout << "X：" << worker[i].trajectory[j].X << " ";
      cout << "Y：" << worker[i].trajectory[j].Y << " ";
      cout << endl;
    }

    cout << "endtime：" << worker[i].endTime << " ";
    cout << "range：" << worker[i].range << " ";
    cout << "score：" << worker[i].score << " ";
    cout << endl;
  }
}

double rad(double d)
{
  return d * PI / 180.0;
}

double GetDistance(double lat1, double lng1, double lat2, double lng2)
{
  //经纬度距离转换为直线距离
  double radLat1 = rad(lat1);
  double radLng1 = rad(lng1);
  double radLat2 = rad(lat2);
  double radLng2 = rad(lng2);
  double a = abs(radLng1 - radLng2);
  double b = abs(radLat1 - radLat2);
  double h = pow(sin(b / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(a / 2), 2);
  double s = 2 * EARTH_RADIUS * sin(sqrt(h)) * 1000;
  return s;
}

//task
double Caculate_mindist(int workerid, int taskid, int (&poi)[Number_Task][Number_Worker])
{
 
  //输入一个task,一个worker,返回task与worker的最小绕路距离,最近的POI点在worker轨迹中的第几个
  vector<POI> Trajectory;                   //定义Trajectory指针
  Trajectory = worker[workerid].trajectory; //指针，访问
  double detour_distance1, mindis = 100000;
  int j = -1;
  for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
  {
    j++;
    detour_distance1 = GetDistance(task[taskid].Y, task[taskid].X, (*it).Y, (*it).X);

    if (detour_distance1 < mindis)
    {
      mindis = detour_distance1;
      poi[taskid][workerid] = j; //记录轨迹中的第j个点是最小的
    }
  }
  return mindis;
}

void Compute_PTPW(int taskid,vector<pair<int, double>> (&PT)[Number_Task],vector<pair<int, double>> (&PW)[Number_Worker], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker])
{
  //计算PT,PW
    for (int j = 0; j < Number_Worker; j++)
    {
      if (task[taskid].Minscore <= worker[j].score) //利用Score先过滤掉一部分工人
      {                                                      //工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
        detour_distance[taskid][j] = Caculate_mindist(j, taskid, poi); //计算每个任务和每个worker之间的最小绕路距离
        if (detour_distance[taskid][j] <= worker[j].range)
        {
          double preference1 = task[taskid].Reward - 2 * detour_distance[taskid][j] * c; //工人的偏好值
          if (preference1 > 0)                                                 //利润大于0
          {
            double preference2 = worker[j].score; //任务的偏好值
            PT[taskid].push_back(make_pair(j, preference2));
            // cout<<taskid<<"\t"<<j<<"偏好计算成功"<<endl;
            #pragma omp critical
            {
            PW[j].push_back(make_pair(taskid, preference1));
            }
          }
          else
          {
            //      cout<<taskid<<"\t"<<j<<"距离太远!"<<endl;
          }
        }
      }
    }
//cout<<"taskid:"<<taskid<<endl;
}

vector<pair<int, double>>::iterator IFTaskExist(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker])
{
  //判断任务在PW的位置
  vector<pair<int, double>>::iterator iter = PW[workerid].begin();
  for (iter; iter < PW[workerid].end(); iter++)
    if ((*iter).first == taskid)
      break;
  return iter;
}


int GetIndex_PT(int workerid, int taskid, vector<pair<int, double>> (&PT)[Number_Task])
{
  //计算workeid在PT中的位置
  int i = 0;
  for (i; i < PT[taskid].size(); i++)
  {
    if (PT[taskid][i].first == workerid)
      break;
  }
  return i;
}

double Caculate_Task_Satisfaction(int (&CW_Task)[Number_Task],vector<pair<int, double>> (&PT)[Number_Task])
{

 double sum=0;
 for(int i=0;i<Number_Task;i++){
  int workerid = CW_Task[i];
  if(workerid != -1){
     int index = GetIndex_PT(workerid,i,PT);
  
     double s = ((PT[i][index].second - 0)/(PT[i][0].second -0));
     sum = sum + s;
   //cout<<i<<"\t"<<workerid<<"\t"<<PT[i].size()<<"\t"<<s<<"\t"<< sum <<endl;
   //cout<<endl;
  }
}
 return sum/Number_Task * 100;
}

double Caculate_Worker_Satisfaction(vector<int> CT_Worker[Number_Worker],vector<pair<int, double>> (&PW)[Number_Worker]){
 double allsum=0;
for(int i=0;i<Number_Worker;i++){
   if(CT_Worker[i].size() !=0 ){
     double sum=0,avg =0;
      for(int j=0; j< CT_Worker[i].size();j++){
      int taskid = CT_Worker[i][j];
      int index = GetIndex_PW(i,taskid,PW);
      double s = (PW[i][index].second - 0)/(PW[i][0].second - 0) ;
      if(s<0)
      cout<<s<<endl;
   //   cout<<s <<endl;
      sum = sum + s;
      
                 // cout<<CT_Worker[i].size()<<endl;
                 // cout<<i<<"\t"<<taskid<<"\t"<<PW[i].size()<<"\t"<<s<<"\t"<< sum <<endl;
                 // cout<<endl;
  }
   avg = sum/CT_Worker[i].size();
   allsum =  allsum + avg;
                 // cout<<allsum<<endl; 
  }

}
 return allsum/Number_Worker *100 ;

}
void ShowDetour_Distance(double (&detour_distance)[Number_Task][Number_Worker],int (&poi)[Number_Task][Number_Worker]){

   for(int i=0; i< Number_Worker; i++)
   for(int j=0; j< Number_Task; j++)
   {
      cout<<"偏移点"<<poi[j][i]<<endl;
      cout<<"工人"<<i<<",任务"<<j<<"的绕路距离:"<<detour_distance[j][i]<<endl;
      cout<<endl;
   }

}
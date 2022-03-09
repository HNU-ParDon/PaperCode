//Input:W,T,PT //PT(w)为任务对工人的偏好列表
//Output:M
//6-8 任务根据他第一个最偏好的工人的距离从近到远提出请求。而不是单纯的一个一个工人的匹配
//去掉那些endtime太小的工人和太大的工人；为每个工人在其附近随机生成一些任务。
//6-19任务： 跑通模拟数据集的代码！！！！！！！
//6-21 11:00 从工人的轨迹点中随机选取，在其半径范围大概为500M的范围内生成任务的位置
//7-20 18:11 成本c=1.0，求任务的偏好时同样需要加入工人的偏好值大于0的约束
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <regex>
#include <string>
#include <cmath>
#include <math.h>
#include <random>
#include <time.h>
#include <Windows.h>
#include <stdlib.h>
#include <sstream>
#include <cstdlib>
#include <omp.h>
using namespace std;

const double c =  1.0/1000;         //单位距离的工人成本
const double speed = 1000;          //工人的移动速度1km/min,60km/h,1km/min
const int Number_Task = 6000;
const int Number_Worker = 1000;
const int Worker_Record = 3000;  
const int Number_Trajectory_Point = 10;    //每个工人取10个轨迹点
const int Capacity = 3;

#define PI 3.1415926535897932384626433832795
#define EARTH_RADIUS 6378.137   //地球半径 KM
 //定义worker的数据结构，定义轨迹的数据结构
 //定义task的数据结构

struct POI
{ //POI点的结构
  double X;
  double Y;
};

struct WORKER
{ //worker的结构
  vector<POI> trajectory; //轨迹,容器tra里面顺序存储了POI点信息,轨迹文件单独读取？
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

WORKER workerRecord[Worker_Record];  //从数据集文件中获得
WORKER worker[Number_Worker];       //Worker数组；
TASK task[Number_Task];             //Task数组；随机生成
static int poi[Number_Task][Number_Worker];              //存储poiid


void ShowMatching(vector<pair<int, int>> &Matching);
double Caculate_mindist(int workerid, int taskid, int (&poi)[Number_Task][Number_Worker]); //声明函数
void Initialize( double (&AD)[Number_Worker], double (&Sumdis)[Number_Worker]);
void Compute_AvailableWorker(int taskid, vector<pair<int, double>> (&PT)[Number_Task], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker]);
bool cmp(pair<int, double> a, pair<int, double> b);
int FindBestWorker(int taskid, vector<int> &AW, vector<pair<int, double>> (&PT)[Number_Task],double(&AD)[Number_Worker], double(&detour_distance)[Number_Task][Number_Worker]);
bool SatisfiedTimeAndDis(int workerid, int taskid, double (&AD)[Number_Worker],  double (&detour_distance)[Number_Task][Number_Worker]);
bool cmp2(pair<int,double> a, pair<int,double> b);
int GetIndex_PT(int workerid, int taskid, vector<pair<int, double>> (&PT)[Number_Task]);
int GetIndex_PW(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]);
double Caculate_Task_Satisfaction(vector<pair<int, int>> (&Matching), vector<pair<int, double>> (&PT)[Number_Task]);
double Caculate_Worker_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PW)[Number_Worker]);
void Compute_AvailableTask(int workerid, vector<pair<int, double>> (&PW)[Number_Worker], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker]);
void ShowDetour_Distance(double (&detour_distance)[Number_Task][Number_Worker],int (&poi)[Number_Task][Number_Worker]);
void ReproduceWorker(WORKER (&worker)[Number_Worker], WORKER (&workerRecord)[Worker_Record],double (&Sumdis1)[Worker_Record],double (&Sumdis)[Number_Worker]);
void Produce_Random_Task_ForTrajectoryPoint(int tasknumber,int trajectory_point);
int FindPreferedWorker(int taskid, vector<int> &AW, double(&AD)[Number_Worker]);
bool SatisfiedDeadline(int workerid, int taskid, double (&AD)[Number_Worker],  double distance);


//声明函数
void Get_Trajectory_locations(WORKER (&workerRecord)[Worker_Record]) ; //生成轨迹位置文件
void Caculate_Sumdist_Trajectory(double (&Sumdis1)[Worker_Record],WORKER (&workerRecord)[Worker_Record]);
void Prodece_Worker_endTime_range_score(WORKER (&worker)[Number_Worker], double (&Sumdis)[Number_Worker],double endtimeX,double rangeX, double scoreX);  //生成worker的其它属性
void showWorker(WORKER (&worker)[Number_Worker]);
void Prodece_Task_Reward_Minscore(TASK (&task)[Number_Task]);   //生成task的其它属性
void showTask(TASK (&task)[Number_Task]);
double rad(double d);
double GetDistance(double lat1, double lng1, double lat2, double lng2);

int main()
{
 double Sumdis1[Worker_Record]= {0}; //计算每个工人的路径长
 double Sumdis[Number_Worker]= {0}; //计算每个工人的路径长
 Get_Trajectory_locations(workerRecord);
 Caculate_Sumdist_Trajectory(Sumdis1,workerRecord);
 /*
    int tong[5]={0};
    for(int i=0;i<Worker_Record;i++)
    {
       if(Sumdis1[i]< 15000)
           tong[0]++;
       else if(Sumdis1[i]< 50000 ) 
           tong[1]++;
      else if(Sumdis1[i]< 70000)
           tong[2]++;
      else if(Sumdis1[i]<100000)
          tong[3]++;
      else if(Sumdis1[i]>160000)
          tong[4]++;
  }
cout<<"总旅行距离<15km的工人数："<<tong[0]<<endl;  //3000：800 ,10个轨迹点
cout<<"总旅行距离<50km的工人数："<<tong[1]<<endl;
cout<<"总旅行距离<70km的工人数："<<tong[2]<<endl;
cout<<"总旅行距离<100km的工人数："<<tong[3]<<endl;
cout<<"总旅行距离>160km的工人数："<<tong[4]<<endl; //3000: 17 ,10个轨迹点
*/
   ReproduceWorker(worker,workerRecord,Sumdis1,Sumdis);
   Prodece_Worker_endTime_range_score(worker,Sumdis,0.4,2000,1);
   //showWorker(worker);
   //ProduceLocationForTask(task,worker); 
   Produce_Random_Task_ForTrajectoryPoint(int(Number_Task/Number_Worker),Number_Trajectory_Point);

   Prodece_Task_Reward_Minscore(task);

//到达时间.
/*
    int tongji[5]={0};
    for(int i=0;i<Number_Worker;i++)
    {
      double planendtime =Sumdis[i]/speed;
    if(planendtime > 10 && planendtime <= 20)
    tongji[0]++;
   if(planendtime > 20 && planendtime <= 30 )
    tongji[1]++;
    if(planendtime > 30 && planendtime <= 40 )
      tongji[2]++;
         if(planendtime > 40 && planendtime <= 50 )
    tongji[3]++;
    if(planendtime > 50  )
      tongji[4]++;
    }
    for(int i=0;i<5;i++)
        cout<<tongji[i]<<endl;
cout<<endl;
*/

  double run_time;   
  LARGE_INTEGER time_start; //开始时间   
  LARGE_INTEGER time_over; //结束时间
  double dqFreq;  //计时器频率  
  LARGE_INTEGER f; //计时器频率 
  QueryPerformanceFrequency(&f);  
  dqFreq=(double)f.QuadPart;
  QueryPerformanceCounter(&time_start); //计时开始
  //showTask(task);    
  //计算worker对Task的偏好列表Pw(T);
  //根据最大绕路距离筛选可用任务;
  //找出每个任务到每条worker轨迹的最近轨迹点求出绕路距离.
   vector<pair<int, double>> PT[Number_Task];
   vector<pair<int, double>> PW[Number_Worker];
   vector<pair<int, int>> Matching;  //总配对
   //map<int, double> PT[Number_Task];  //存储任务的偏好列表
   static double  detour_distance[Number_Task][Number_Worker]; 
  
  //存储detour_distance

  double  AD[Number_Worker];          //AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
  Initialize(AD,Sumdis);

/*
  for(int i=0; i< Number_Worker; i++)
    for(int j=0; j<Number_Task; j++)
  {
   if(Caculate_mindist(i,j,poi) <10)
   cout<<"i:"<<i<<"\t"<<"j:"<<j<<"\t"<<"mindist:"<<Caculate_mindist(i,j,poi);
 }
 */

 //#pragma omp parallel for num_threads(2)    //4线程的并行处理
 /* for (int i = 0; i < Number_Task; i++)
  {
    Compute_AvailableWorker(i, PT, detour_distance, poi);    //计算可用工人及偏好值并加入到偏序列表
    sort(PT[i].begin(), PT[i].end(), cmp);                  //对任务的偏序列表排序
  }
  */
  //cout<<"并行处理结束！"<<endl;

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
   //   cout<<"任务"<<i<<"的偏好列表:"<<endl;
  //    for(int j=0; j<PT[i].size();j++){
   //      cout<<"工人id："<<PT[i][j].first<<",偏好值："<<PT[i][j].second<<endl;
   //       cout<<endl;
     }
     
   }
     cout<<"没有偏好列表的任务数:"<<sum<<endl;
     cout<<endl;
   */

/*
 int sum1 = 0;
  for (int i = 0; i < Number_Worker; i++)
  {
    if (PW[i].size() == 0)
    {
      cout << i << "工人的偏好列表为空！！！！" << endl;
      sum1++;
    }
    else
   {
    //   cout<<"工人分数："<< worker[i].score<<endl;
    //   cout<<"工人"<<i<<"的偏好列表共有任务数:"<<PW[i].size()<<endl;
    //    for(int j=0; j<PW[i].size();j++)
    //     {  
    //       cout<<"任务id："<<PW[i][j].first<<"任务报酬："<<task[PW[i][j].first].Reward<<"任务绕路："<<detour_distance[PW[i][j].first][i]<<"任务分数约束："<<task[PW[i][j].first].Minscore<<",偏好值："<<PW[i][j].second<<endl;
    //        cout<<endl;
    //      } 
   }
   //cout<<endl;
  }
  cout << "偏好列表为空的工人总数为：" << sum1 << endl;
  cout<< endl;
  */

//cout<<"距离:"<<detour_distance[222][0]<<"\t"<<detour_distance[1150][0]<<"\t"<<detour_distance[1256][0]<<"\t"<<detour_distance[1257][0]<<endl;
//cout<<endl;


/*  int sum1=0;
 for(int i=0 ;i< Number_Task ;i++)
   {
     if(PT[i].size() > 0){
       cout<<i<<"任务的偏好列表为不为空！！！！"<<endl;
       sum1++;
     }
   }
cout<<"偏好列表不为空的任务总数为："<<sum1<<endl;
*/

   
  //输出mindist测试
/*for(int i=0; i<Number_Worker ;i++)
   for(int j=0; j<Number_Task;j++)
   {
   double dd=Caculate_mindist(i,j,poi);
   if( dd < 10 ){
      cout<<"i:"<<i<<","<<"j:"<<j<<","<<"poi:"<<poi[j][i]<<"mindist:"<< dd<<endl;
      cout<<"worker'X:"<<worker[i].trajectory[poi[j][i]].X<<"worker'Y:"<<worker[i].trajectory[poi[j][i]].Y<<endl;
      cout<<"task'X:"<<task[j].X<<"task'Y:"<<task[j].Y<<endl<<endl;
   }
   }
   */
//ShowDetour_Distance(detour_distance,poi);

  //实现贪心算法
  vector<int> AW; //定义可用工人的集合为vector变量
  for (int i = 0; i < Number_Worker; i++)
  {
    AW.push_back(i);
  }


  for (int i = 0; i < Number_Task; i++) //random_shuffle(begin,end):随机打乱数组的元素位置
  {
    if (AW.size() != 0)   //当还有可用工人时，可对任务继续进行匹配
     {                               

     int workerid = -1;
     // workerid = FindBestWorker(i, AW, PT,AD,detour_distance); //偏好列表里找到了最前的可用工人编号，且需要满足加入任务在其预算范围内
      workerid = FindPreferedWorker(i, AW, AD);
      if (workerid != -1)                       //找到可匹配的
      {
        //  cout<<i<<"任务满足时间和绕路距离约束！"<<workerid<<endl;
          Matching.push_back(make_pair(i, workerid));                               //匹配;
        //  AD[workerid] = AD[workerid] - 2 * detour_distance[i][workerid];           //轨迹中的最后一个点
         
        int assignedtask =0;
       for(int i=0;i<Matching.size();i++){
        int workerid2= Matching[i].second;
           if(workerid2 == workerid)
                assignedtask ++;
       }

          if ( assignedtask  == Capacity  )       //工人容量已满时 ,移除工人
          {
            for (vector<int>::iterator it = AW.begin(); it != AW.end(); ++it) //移除工人
            {
              if (*it == workerid)
              {
                it = AW.erase(it);
              // cout<<workerid<<"工人已移除！"<<endl;
              // cout<<endl;
                break;
              }
            }
          }
      }
      else  
      {
     //   cout<<i<<"任务无可加入的工人！"<<endl;
     //   cout<<endl;
      }


  }
          else
          break; //无可用工人则跳出循环，任务匹配结束。
  }

  QueryPerformanceCounter(&time_over); //计时结束 
  run_time=(time_over.QuadPart-time_start.QuadPart)/dqFreq;//乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒    
  printf("\nSumArrRow run_time：%.4fs\n",run_time);

   for (int i = 0; i < Number_Task; i++)
  {
    Compute_AvailableWorker(i, PT, detour_distance, poi);    //计算可用工人及偏好值并加入到偏序列表
    sort(PT[i].begin(), PT[i].end(), cmp);                  //对任务的偏序列表排序
  }
  
  for (int i = 0; i < Number_Worker; i++)
  {
    Compute_AvailableTask(i, PW, detour_distance, poi); //计算可用任务及偏好值并加入到偏序列表
    sort(PW[i].begin(), PW[i].end(), cmp);                //对工人的偏序列表排序
  }

ShowMatching(Matching); //输出配对
//   cout<<"range:"<<TestEndtimeX<<endl;
cout<<"任务的平均满意度："<<Caculate_Task_Satisfaction(Matching,PT)<<endl;
cout<<"工人的平均满意度："<<Caculate_Worker_Satisfaction(Matching,PW)<<endl;
cout<<endl;
//}
      return 0;
  }

    void Initialize( double(&AD)[Number_Worker], double (&Sumdis)[Number_Worker])
    { //初始化AD
      for (int j = 0; j < Number_Worker; j++)
      {    
       AD[j] = worker[j].endTime * speed - Sumdis[j];
      }

/*
//可用绕路距离.
    int tongji[8]={0};
    for(int i=0;i<Number_Worker;i++)
    {
      double ad = AD[i];
   if(ad > 10000 && ad <= 15000 )
    tongji[0]++;
   if(ad > 15000 && ad <= 20000 )
    tongji[1]++;
    if(ad > 20000 && ad <= 25000 )
      tongji[2]++;
    if(ad > 25000 && ad <= 30000 )
      tongji[3]++;
   if(ad > 30000 && ad <= 35000 )
      tongji[4]++;
         if(ad > 35000 && ad <= 40000 )
      tongji[5]++;
  if(ad > 40000  && ad <= 45000)
       tongji[6]++;
      if(ad > 45000  && ad <= 100000)
       tongji[7]++;
    }
    for(int i=0;i<8;i++)
        cout<<tongji[i]<<endl;
        cout<<endl;
        */
    }

    void ShowMatching(vector<pair<int, int>> & Matching)
    {
   /*   cout << "总配对如下:" << endl;
      cout << "任务id"
           << "\t"
           << "工人id" << endl;
      for (vector<pair<int, int>>::iterator it = Matching.begin(); it != Matching.end(); it++)
      {
        cout << (*it).first << "\t" << (*it).second << endl;
      }
      */
  map<int,int> worker_count;
  for(int i=0;i<Matching.size();i++){
   int workerid = Matching[i].second;
   int taskid = Matching[i].first ;
        worker_count[workerid]++;
}
 cout << "任务总匹配数为:" << Matching.size() << endl;
 cout<<"匹配到的工人数："<<worker_count.size() <<endl;

    }

void Compute_AvailableWorker(int taskid, vector<pair<int, double>>(&PT)[Number_Task], double(&detour_distance)[Number_Task][Number_Worker], int(&poi)[Number_Task][Number_Worker])
    { 
      //计算taskid任务的可用工人并加入偏序列表
      for (int j = 0; j < Number_Worker; j++)
      {
        if (task[taskid].Minscore <= worker[j].score)
        {                                                                  //工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
          detour_distance[taskid][j] = Caculate_mindist(j, taskid, poi); //计算每个任务和每个worker之间的最小绕路距离
          if (detour_distance[taskid][j] <= worker[j].range)
          {                                                 //当任务的绕路距离小于工人的最大绕路距离，计算偏好
                                                            //  double preference= worker[j].trajectory[poiid].poitime+detour_distance/speed;      //偏好值单任务的等待时间
            double  preference2 = task[taskid].Reward - (2 * detour_distance[taskid][j] * c);              //偏好值为score
           if (preference2 > 0)
            {
              double preference = worker[j].score;            //偏好值为score
              PT[taskid].push_back(make_pair(j, preference)); //I任务的j worker对应的preference
              //cout<<taskid<<"\t"<<j<<"偏好计算成功"<<endl;
            }
          }
          else{
             //cout<<taskid<<"\t"<<j<<"距离太远!"<<detour_distance[taskid][j]<<endl;
          }
             //cout<<endl;
        }

      }
      

    }

    bool cmp(pair<int, double> a, pair<int, double> b) //比较函数
    {
      return a.second > b.second;
    }


    int FindBestWorker(int taskid, vector<int> &AW, vector<pair<int, double>>(&PT)[Number_Task],double(&AD)[Number_Worker], double(&detour_distance)[Number_Task][Number_Worker])
    {
      int aw_workerid = -1;
      for (int j = 0; j < PT[taskid].size(); j++)
      {   //遍历该任务的偏好列表
          //vector< pair<int,double> > *p= &PT[i];
       int  workerid = PT[taskid][j].first;
       if(find(AW.begin(),AW.end(),workerid)!=AW.end())     //workerid工人属于AW     
          if (SatisfiedTimeAndDis(workerid, taskid, AD, detour_distance))
           { 
             aw_workerid = workerid;
              break;
           }
        }

      return aw_workerid;
    }

    bool SatisfiedTimeAndDis(int workerid, int taskid, double(&AD)[Number_Worker], double(&detour_distance)[Number_Task][Number_Worker])
    {

      double ADD = AD[workerid] - 2* detour_distance[taskid][workerid];               //轨迹中的最后一个点
      if (ADD >= 0)
        return 1;
      else
        return 0;
    }
  //void Compute_Preference_list(TASK task, WORKER worker[Number_Worker],int *PT){    //输入一个task,全部的worker，求出task对worker的偏好列表,以及任务的偏好列表PT
  //4.12 21:36 OMG,文件的读取存入响应的数据结构还没实现

  /*  for(int i=0;i<Number_Task;i++)   //输出测试
	{
		printf("%.6f\t",task[i].X);
		printf("%.6f\n",task[i].Y);
	    // cout<<task[i].X<<"\t"<<task[i].Y<<endl;
	}
	*/

void Get_Trajectory_locations(WORKER (&workerRecord)[Worker_Record]){  
    int t=1, w=1; 
    while(w <= Worker_Record){
     stringstream ss; 
     string str;
     ss<<t;
     ss>>str;
	 ifstream in("F:\\CPP\\dataset\\T-Drive\\"+str+".txt"); 
   if(in){
   vector<POI> trajectory;
   string line;
   int tranum = 0;
  while( tranum < 40 )  //按行读取
  {
    getline(in, line);
    if(tranum % 4 == 0){
    POI poi;
  char *s_input =(char *) line.c_str();
  const char  * split= ",";
  char *p =strtok(s_input,split);
  double a;
  int i=0;
  while(p != NULL){
  a = atof(p);
   if(a == 0 )
      { 
        cout<<t<<endl;
        cout<<endl;
      }
       if(i==2)
       {
         if(a <115.7 && a>117.4) //北京的经度范围
          { cout<<t<<endl;
            cout<<endl;
          }
        poi.X = a; 
       }
       if(i==3)  
       {
         if(a <39.4 && a>41.6) //北京的纬度范围
         {
          cout<<t<<endl;
          cout<<endl;
         }
       poi.Y = a; 
       }   
  p = strtok(NULL,split);
  i++;
}
workerRecord[w-1].trajectory.push_back(poi);
    }
tranum++;
}
 in.close();
w++;
  }
else{
  // cout<<"文件不存在！"<<endl;
}
t++;

   }
cout<<"工人的轨迹文件读取完成！"<<endl;
//cout<<endl;

int p=0,z=0;
  vector<int> numbb;
for(int i=0;i <Worker_Record;i++)    //输出trajectory_location
{
 // cout<<"工人"<<i<<"的轨迹点数量："<<workerRecord[i].trajectory.size()<<endl;	

  for(int j=0; j< workerRecord[i].trajectory.size();j++)
  {
   // cout<<"("<<workerRecord[i].trajectory[j].X<<","<<workerRecord[i].trajectory[j].Y<<")"<<" "<<endl;
  if(workerRecord[i].trajectory[j].X==0 || workerRecord[i].trajectory[j].Y==0)
       {
         z++;
         numbb.push_back(i);
       }
  }
 // cout<<endl;	
 if(workerRecord[i].trajectory.size()==0)
    p++;
}
//cout<<"轨迹点为0的工人数目"<<p<<endl;
//cout<<endl;
if(z !=0)
{
  cout<<"轨迹点经纬度为0的工人有几个人："<<z<<endl;
  for(int j=0;j<numbb.size();j++){
    cout<<"工人的序号为:"<<numbb[j]<<endl;
  }
}
cout<<endl;

}

  void Caculate_Sumdist_Trajectory(double (&Sumdis1)[Worker_Record],WORKER (&workerRecord)[Worker_Record]){
    for(int i=0; i < Worker_Record;i++){
	     double sum=0;
       for(int j=0; j < workerRecord[i].trajectory.size()-1; j++){
      sum = sum+ GetDistance( workerRecord[i].trajectory[j].Y,workerRecord[i].trajectory[j].X ,workerRecord[i].trajectory[j+1].Y, workerRecord[i].trajectory[j+1].X );  
//	  cout<<sum<<endl;
	   }
	   Sumdis1[i] = sum;
	//   cout<<i<<" "<<"all:" << Sumdis[i] <<endl;
	}

}

void Prodece_Task_Reward_Minscore(TASK (&task)[Number_Task]){    //随机生成task的reward和minscore
     default_random_engine e1,e2;
     //uniform_real_distribution<double> u1(60,100);  //score,均匀的生成
  	 //uniform_int_distribution<unsigned> u2(1,10);  //reward
     normal_distribution<double> u1(80,10);
     normal_distribution<double> u2(8,1);
     for(int i = 0; i < Number_Task; ++i){
            task[i].Minscore = u1(e1);
            task[i].Reward = u2(e2);
      //      cout << "score:"<< task[i].Minscore << endl;
		//	cout <<"reward:"<< task[i].Reward << endl;
	 }
      
}

void Prodece_Worker_endTime_range_score(WORKER (&worker)[Number_Worker],double (&Sumdis)[Number_Worker],double endtimeX,double rangeX, double scoreX)
{  
	//设置参数endtime,range,score,Maxdetour;
     default_random_engine e;
     //uniform_real_distribution<double> u(60,100);  //score 均匀分布
     normal_distribution<double> u(80,10);  //score 均匀分布
     
     for(int i = 0; i < Number_Worker; ++i){
      worker[i].endTime = (Sumdis[i] / speed)  * (1 + endtimeX);
			worker[i].range = rangeX;
			worker[i].score = u(e)*scoreX;
    /*    cout <<"endTime: "<< worker[i].endTime << endl;
	     cout <<"sumdis: "<< Sumdis[i] << endl;
		cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
		cout <<"range: "<< worker[i].range << endl;
		cout <<"score: "<< worker[i].score << endl;
		*/
	    }
}

void showTask(TASK (&task)[Number_Task]){

   for(int i=0; i< Number_Task; i++){
      cout<<"id: "<<i<<" ";
      cout<<"X："<<task[i].X<<" ";
      cout<<"Y："<<task[i].Y<<" ";
      cout<<"minscore："<<task[i].Minscore<<" ";
      cout<<"reward："<<task[i].Reward<<" ";
      cout<<endl;
   }


}

void showWorker(WORKER (&worker)[Number_Worker]){
   for(int i=0; i< Number_Worker; i++){
      cout<<"工人id: "<<i<<endl;
	    for(int j=0; j<worker[i].trajectory.size(); j++){
			 cout<<"X："<<worker[i].trajectory[j].X<<" ";
			 cout<<"Y："<<worker[i].trajectory[j].Y<<" ";
			 cout<<endl;
	      }
      cout<<"endtime："<<worker[i].endTime <<" ";
      cout<<"range："<<worker[i].range <<" ";
	    cout<<"score："<<worker[i].score<<" ";
     cout<<endl;
   }

}

double Caculate_mindist(int workerid, int taskid, int(&poi)[Number_Task][Number_Worker])
    {                       //输入一个task,一个worker,返回task与worker的最近绕路距离,以及最近的POI点在worker轨迹中的第几个
      vector<POI> Trajectory;                   //定义Trajectory指针
      Trajectory = worker[workerid].trajectory; //指针，访问
      double detour_distance, mindis =  100000;
       int   j=-1;
        for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
        {   
          j++;
        detour_distance = GetDistance( task[taskid].Y, task[taskid].X , (*it).Y,  (*it).X );
		    if (detour_distance < mindis) 
          {
            mindis = detour_distance;
            poi[taskid][workerid] = j; //记录轨迹中的第j个点是最小的
          }
        }
      return mindis;
    }

 double rad(double d)
{
   return d * PI / 180.0;
}

 double GetDistance(double lat1, double lng1, double lat2, double lng2)
{
   double radLat1 = rad(lat1);
   double radLng1 = rad(lng1);
   double radLat2 = rad(lat2);
   double radLng2 = rad(lng2);
   double a =abs(radLng1 - radLng2);
   double b =abs(radLat1 - radLat2);
   double h = pow(sin(b/2),2)+ cos(lat1) * cos(lat2) * pow(sin(a/2),2);
   double s = 2 * EARTH_RADIUS * sin(sqrt(h)) *1000 ;

  return s;
}


bool cmp2( pair<int,double> a, pair<int,double> b){

return a.second< b.second;

}


double Caculate_Task_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PT)[Number_Task])
{
 double sum=0;
 for(int i=0;i < Matching.size();i++){
    int workerid =Matching[i].second ;
    int taskid =Matching[i].first ;
    int index = GetIndex_PT(workerid,taskid,PT) ;
 // if(index == PT[taskid].size())
   //  cout<<"未在任务中找到工人！"<<endl;
    double s =  ((PT[taskid][index].second - 0)/(PT[taskid][0].second -0))  ;
    sum = sum + s ;
   //cout<<taskid<<"\t"<<workerid<<"\t"<<index<<"\t"<<PT[taskid].size()<<"\t"<<s<<"\t"<< sum <<endl;
   //cout<<endl;
  }
 return sum/Number_Task *100 ;
}

double Caculate_Worker_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PW)[Number_Worker]){
vector<int>  CT_Worker[Number_Worker];
 for(int i=0;i<Matching.size();i++){  
   int workerid=Matching[i].second;
   int taskid =Matching[i].first;
   CT_Worker[workerid].push_back(taskid);
 }

 double allsum=0;
  for(int i=0;i<Number_Worker;i++){
     if(CT_Worker[i].size() !=0 ){
     double sum=0,avg=0;
      for(int j=0; j< CT_Worker[i].size();j++){
      int taskid = CT_Worker[i][j];
      int index = GetIndex_PW(i,taskid,PW);
   //   if(index == PW[i].size())
   //  cout<<"未在工人中找到任务！"<<endl;
      double s = (PW[i][index].second - 0)/(PW[i][0].second -0) ;
      sum = sum + s;
    //  cout<<CT_Worker[i].size()<<endl;
  //   cout<<i<<"\t"<<taskid<<"\t"<<index<<"\t"<<PW[i].size()<<"\t"<<s<<"\t"<< sum <<endl;
   //  cout<<endl;
  }
  avg =sum/CT_Worker[i].size();
  allsum =allsum +avg;
  //cout<<allsum<<endl;
  
  }

}
 return allsum/Number_Worker * 100;

}

void Compute_AvailableTask(int workerid, vector<pair<int, double>>(&PW)[Number_Worker], double(&detour_distance)[Number_Task][Number_Worker], int(&poi)[Number_Task][Number_Worker])
  {     //计算taskid任务的可用任务并加入偏序列表
    for (int j = 0; j < Number_Task; j++)
    {
      if (task[j].Minscore <= worker[workerid].score)
      {                                                                      //工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
        detour_distance[j][workerid]= Caculate_mindist(workerid, j, poi); //计算每个任务和每个worker之间的最小绕路距离
        if (detour_distance[j][workerid]<= worker[workerid].range)
        {                                                    //当任务的绕路距离小于工人的最大绕路距离，计算偏好
                                                             //  double preference= worker[j].trajectory[poi].poitime+detour_distance/speed;      //偏好值单任务的等待时间
        double  preference = task[j].Reward - (2 * detour_distance[j][workerid] * c);              //偏好值为score
        if (preference > 0)
        {
        PW[workerid].push_back(make_pair(j, preference)); //I任务的j worker对应的preference
        }
        }
      }
    }
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
int GetIndex_PT(int workerid, int taskid, vector<pair<int, double>> (&PT)[Number_Task])
{
  int i = 0;
  for (i; i < PT[taskid].size(); i++)
  {
    if (PT[taskid][i].first == workerid)
      break;
  }
  return i;
}

void ShowDetour_Distance(double (&detour_distance)[Number_Task][Number_Worker],int (&poi)[Number_Task][Number_Worker]){
   for(int i=0;i<Number_Task;i++)
   for(int j=0;j<Number_Worker;j++)
   {
     cout<<"偏移点"<<poi[i][j]<<endl;
    cout<<"任务"<<i<<",工人"<<j<<"的绕路距离:"<<detour_distance[i][j]<<endl;
    cout<<endl;
   }
   
  /*
   double avaiable[Number_Worker];
for(int i=0;i<Number_Worker;i++)
{
   for(int j=0;j<Number_Task;j++)
   {
  if(detour_distance[i][j] < 2000)
      avaiable[i]++;
   }
}

int tongji[5]={0};
 for(int i=0;i<Number_Worker;i++)
{
    double min = avaiable[i];
   if(min > 500 && min <= 1000 )
        tongji[0]++;
   if(min > 1000 && min <= 1200 )
        tongji[1]++;
    if(min > 1200 && min <= 1400 )
        tongji[2]++;
    if(min > 1400 && min <= 1600 )
        tongji[3]++;
}
    for(int i=0;i<5;i++)
        cout<<tongji[i]<<endl;
   cout<<endl;
   */
}

void  ReproduceWorker(WORKER (&worker)[Number_Worker], WORKER (&workerRecord)[Worker_Record],double (&Sumdis1)[Worker_Record],double (&Sumdis)[Number_Worker]){
  int j=0;
  for(int i=0;i<Worker_Record;i++)
  {
    if(j<Number_Worker){

    if(Sumdis1[i]>= 15000 && Sumdis1[i]<=160000)
    {
     worker[j] = workerRecord[i];
     Sumdis[j] = Sumdis1[i];
  /*  cout<<i<<"的原始信息："<<workerRecord[i].endTime<<"\t"<<workerRecord[i].range<<"\t"<<workerRecord[i].score<<"\t"<<workerRecord[i].trajectory.size()<<endl;   
    cout<< Sumdis1[i]<<endl;
    cout<<endl;
    cout<<j<<"工人的信息："<<worker[j].endTime<<"\t"<<worker[j].range<<"\t"<<worker[j].score<<"\t"<<worker[j].trajectory.size()<<endl;     
    cout<< Sumdis[j]<<endl;
    cout<<endl;
    */
   j++;
    }
    }
  }
   cout<<"worker总数："<<j<<endl;
}


void Produce_Random_Task_ForTrajectoryPoint(int tasknumber,int trajectory_point){
     default_random_engine e;
     int tasknum=0;
     for(int i=0;i<Number_Worker;i++)
      for(int j=0; j< tasknumber;j++)
       {
         int t = rand()% trajectory_point;  //返回0-39的数字
         uniform_real_distribution<double> u2(worker[i].trajectory[t].X - 0.04,worker[i].trajectory[t].X + 0.04 );    //经度范围
         uniform_real_distribution<double> u3(worker[i].trajectory[t].Y - 0.04,worker[i].trajectory[t].Y + 0.04 );   //纬度范围
         task[tasknum].X =  u2(e);
         task[tasknum].Y =  u3(e);
        // cout << task[tasknum].X <<endl;
        // cout << task[tasknum].Y <<endl;     
       tasknum++;
       }
        while(tasknum < Number_Task){  //随机选工人，随机选取轨迹点
          int wi = rand()% Number_Worker;  //返回0-NumberWorker-1的数字
          int ti = rand()% Number_Trajectory_Point;  //返回0-Number_Trajectory_Point-1的数字
          uniform_real_distribution<double> u2(worker[wi].trajectory[ti].X - 0.005,worker[wi].trajectory[ti].X + 0.05 );    //经度范围
          uniform_real_distribution<double> u3(worker[wi].trajectory[ti].Y - 0.005,worker[wi].trajectory[ti].Y + 0.05 );   //纬度范围
          task[tasknum].X = u2(e) ;
          task[tasknum].Y = u3(e) ;
          // cout<<tasknum<<"任务"<<"的经纬度："<< task[tasknum].X <<","<<task[tasknum].Y<<endl;
          tasknum++;
       }
       cout<<"任务总数："<<tasknum<<endl;

}


int FindPreferedWorker(int taskid, vector<int> &AW, double(&AD)[Number_Worker])
    {
      //找最偏好的工人
      int best_workerid = -1;
      double maxpre = - __DBL_MAX__;
      double best_distance = 0;
      for (int j = 0; j < AW.size(); j++)
      {   //遍历所有可用工人
        int  workerid = AW[j];
        if(task[taskid].Minscore <= worker[workerid].score)        //worker分数满足大于MinScore
          {
            double preference = worker[workerid].score ;
            if ( preference > maxpre)   //满足分数的约束,********
            {
               double dist = Caculate_mindist(workerid,taskid,poi);
               if(worker[workerid].range >= dist)     //计算绕路距离距离小于range 
               {
                   if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束
                   {
                    double  preference2 = task[taskid].Reward - (2 * dist * c) ;
                    if( preference2 >0 )
                    {
                     maxpre = preference ;
                     best_workerid = workerid ;
                     best_distance = dist ;
                    }
                  }
               }
            }
         }
      }
      
      if(best_workerid != -1)
            AD[best_workerid] = AD[best_workerid] - 2 * best_distance; //计算剩余AD[workerid]
      return best_workerid;
    }

    
    bool SatisfiedDeadline(int workerid, int taskid, double(&AD)[Number_Worker], double distance)
    {

      double ADD = AD[workerid] - 2* distance;               //可用绕路距离
      if (ADD >= 0)
        return 1;
      else
        return 0;
    }
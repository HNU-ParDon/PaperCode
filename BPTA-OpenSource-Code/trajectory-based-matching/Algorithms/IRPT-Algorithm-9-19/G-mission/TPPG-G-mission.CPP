
//Input:W,T,PT //PT(w)为任务对工人的偏好列表
//Output:M
//6-8 任务根据他第一个最偏好的工人的距离从近到远提出请求。而不是单纯的一个一个工人的匹配

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

using namespace std;

const double c =  1.0 ;      //单位距离的工人成本
const double speed = 1 ;     //工人的移动速度1km/min,60km/h,1km/min
const int Number_Task = 700 ;
const int Number_Worker = 200 ;
const int Number_BusStop = 4346 ;
const int Capacity = 1 ;

#define PI 3.1415926535897932384626433832795
#define EARTH_RADIUS 6378.137 //地球半径 KM
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

WORKER worker[Number_Worker];    //Worker数组；从数据集文件中获得
TASK task[Number_Task];          //Task数组；从数据集文件中获得
  //double  task_watitime[Number_Task][Number_Worker]={0};
static  int poi[Number_Task][Number_Worker];              //存储poiid

void ShowMatching(vector<pair<int, int>> &Matching);
double Caculate_mindist(int workerid, int taskid, int (&poi)[Number_Task][Number_Worker]); //声明函数
void Initialize( double (&AD)[Number_Worker], double (&Sumdis)[Number_Worker]);
void Compute_AvailableWorker(int taskid, vector<pair<int, double>> (&PT)[Number_Task], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker]);
bool cmp(pair<int, double> a, pair<int, double> b);
int FindBestWorker(int taskid, vector<int> &AW, vector<pair<int, double>> (&PT)[Number_Task],double(&AD)[Number_Worker], double(&detour_distance)[Number_Task][Number_Worker]);
bool SatisfiedTimeAndDis(int workerid, int taskid, double (&AD)[Number_Worker],  double (&detour_distance)[Number_Task][Number_Worker]);
void SortTask(TASK (&task)[Number_Task]);
bool cmp2(pair<int,double> a, pair<int,double> b);
int GetIndex_PT(int workerid, int taskid, vector<pair<int, double>> (&PT)[Number_Task]);
int GetIndex_PW(int workerid, int taskid, vector<pair<int, double>> (&PW)[Number_Worker]);
double Caculate_Task_Satisfaction(vector<pair<int, int>> (&Matching), vector<pair<int, double>> (&PT)[Number_Task]);
double Caculate_Worker_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PW)[Number_Worker]);
void Compute_AvailableTask(int workerid, vector<pair<int, double>> (&PW)[Number_Worker], double (&detour_distance)[Number_Task][Number_Worker], int (&poi)[Number_Task][Number_Worker]);
void ShowDetour_Distance(double (&detour_distance)[Number_Task][Number_Worker],int (&poi)[Number_Task][Number_Worker]);
int FindPreferedWorker(int taskid, vector<int> &AW, double(&AD)[Number_Worker]);
bool SatisfiedDeadline(int workerid, int taskid, double (&AD)[Number_Worker],  double distance);

//声明函数
void ReadLocationForTask(TASK (&task)[Number_Task]);
void Prodece_Task_Reward_Minscore(TASK (&task)[Number_Task]);   //生成task的其它属性
void showTask(TASK (&task)[Number_Task]);
void Get_Trajectory_locations(WORKER (&worker)[Number_Worker]) ; //生成轨迹位置文件
void Caculate_Sumdist_Trajectory(double (&Sumdis)[Number_Worker],WORKER (&worker)[Number_Worker]);
void Prodece_Worker_endTime_range_score(WORKER (&worker)[Number_Worker], double (&Sumdis)[Number_Worker],double endtimeX,double rangeX, double scoreX);  //生成worker的其它属性
void showWorker(WORKER (&worker)[Number_Worker]);
double rad(double d);
double GetDistance(double lat1, double lng1, double lat2, double lng2);

int main()
{
   // for(double TestEndtimeX = 0.1;  TestEndtimeX < 1; TestEndtimeX= TestEndtimeX + 0.1)
  //{
  	double Sumdis[Number_Worker]= {0};
	//int poi[Number_Task][Number_Worker];
    ReadLocationForTask(task);
  	Prodece_Task_Reward_Minscore(task);
	  //showTask(task);
    Get_Trajectory_locations(worker);
   // showWorker(worker);
  	Caculate_Sumdist_Trajectory(Sumdis,worker);

/*
//到达时间.
    int tongji[6]={0};
    for(int i=0;i<Number_Worker;i++)
    {
      double planendtime =Sumdis[i]/speed;
    if(planendtime <= 5)
    tongji[0]++;
    if(planendtime > 5 && planendtime <= 10)
    tongji[1]++;
   if(planendtime > 10 && planendtime <= 15)
    tongji[2]++;
   if(planendtime > 15 && planendtime <= 20 )
    tongji[3]++;
   if(planendtime > 20 && planendtime <= 25 )
    tongji[4]++;
    if(planendtime > 25 && planendtime <= 30 )
      tongji[5]++;
    }
    for(int i=0;i<6;i++)
        cout<<tongji[i]<<endl;
*/

Prodece_Worker_endTime_range_score(worker,Sumdis, 0.3, 1, 1);
//showWorker(worker);
  //计算worker对Task的偏好列表Pw(T)；
  //根据最大绕路距离筛选可用任务；
  //找出每个任务到每条worker轨迹的最近轨迹点求出678绕路距离.

 double run_time;   
 LARGE_INTEGER time_start; //开始时间   
 LARGE_INTEGER time_over; //结束时间
double dqFreq;  //计时器频率  
LARGE_INTEGER f; //计时器频率  
QueryPerformanceFrequency(&f);  
dqFreq=(double)f.QuadPart;
QueryPerformanceCounter(&time_start); //计时开始

  vector<pair<int, double>> PT[Number_Task];
  vector<pair<int, double>> PW[Number_Worker];
  vector<pair<int, int>> Matching; //总配对
  //map<int, double> PT[Number_Task];  //存储任务的偏好列表
  static double detour_distance[Number_Task][Number_Worker]; 
  
 //存储detour_distance
  
  double  AD[Number_Worker];          //AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
  Initialize(AD,Sumdis);
/*
for(int i=0; i< Number_Worker ;i++)
   for(int j=0; j<Number_Task; j++)
   {
    if(Caculate_mindist(i,j,poi) <10)
      cout<<"i:"<<i<<"\t"<<"j:"<<j<<"\t"<<"mindist:"<<Caculate_mindist(i,j,poi);
 }
 */

 //#pragma omp parallel for num_threads(4) 

 /* for (int i = 0; i < Number_Task; i++)
  {
    Compute_AvailableWorker(i, PT, detour_distance, poi); //计算可用工人及偏好值并加入到偏序列表
    sort(PT[i].begin(), PT[i].end(), cmp);                  //对任务的偏序列表排序
  }
    */
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

/*
 int sum1 = 0;
  for (int i = 0; i < Number_Worker; i++)
  {
    if (PW[i].size() == 0)
    {
  //    cout << i << "工人的偏好列表为空！！！！" << endl;
      sum1++;
    }
    else
   {
     
    cout<<"工人分数："<< worker[i].score<<endl;
     cout<<"工人"<<i<<"的偏好列表共有任务数:"<<PW[i].size()<<endl;
     for(int j=0; j<PW[i].size();j++)
       {  
         cout<<"任务id："<<PW[i][j].first<<"任务报酬："<<task[PW[i][j].first].Reward<<"任务绕路："<<detour_distance[PW[i][j].first][i]<<"任务分数约束："<<task[PW[i][j].first].Minscore<<",偏好值："<<PW[i][j].second<<endl;
         cout<<endl;
       }
       
   }
  }
  cout << "偏好列表为空的工人总数为：" << sum1 << endl;
  cout<< endl;
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
      int workerid=-1;
      //workerid = FindBestWorker(i, AW, PT,AD,detour_distance); //偏好列表里找到了最前的可用工人编号，且需要满足加入任务在其预算范围内
        workerid = FindPreferedWorker(i, AW, AD);
      if (workerid != -1)                       //找到可匹配的
      {
          //cout<<i<<"任务满足时间和绕路距离约束！"<<workerid<<endl;
          Matching.push_back(make_pair(i, workerid));                               //匹配;
         // AD[workerid] = AD[workerid] - 2 * detour_distance[i][workerid];           //轨迹中的最后一个点      
    
       int assignedtask =0;
       for(int i=0;i<Matching.size();i++){
        int workerid2= Matching[i].second;
           if(workerid2 == workerid)
                assignedtask ++;
       }
          //cout<<workerid<<"已匹配的任务数："<<assignedtask<<endl;
          if ( assignedtask  == Capacity )         //*******可优化                 //工人容量已满，移除
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
  printf("\nSumArrRow run_time：%fs\n",run_time);

for (int i = 0; i < Number_Task; i++)
  {
    Compute_AvailableWorker(i, PT, detour_distance, poi); //计算可用工人及偏好值并加入到偏序列表
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
  //可用绕路距离.
  /*
    int tongji[8]={0};
    for(int i=0;i<Number_Worker;i++)
    {
      double ad = AD[i];
   if(ad > 1000 && ad <= 2000)
    tongji[0]++;
   if(ad > 2000 && ad <= 3000 )
    tongji[1]++;
   if(ad > 3000 && ad <= 4000 )
    tongji[2]++;
    if(ad > 4000 && ad <= 5000 )
      tongji[3]++;
    if(ad > 5000 && ad <= 6000 )
      tongji[4]++;
  if(ad > 6000 &&  ad <= 7000)
      tongji[5]++;
    if(ad > 7000 &&  ad <= 8000)
      tongji[6]++;
     if(ad > 8000 &&  ad <= 9000)
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
 cout << "总的任务匹配数为:" << Matching.size() << endl;

 map<int,int> worker_count;
  for(int i=0;i <Matching.size();i++){
   int workerid = Matching[i].second ;
   if(workerid != -1)
      worker_count[workerid]++;
   }
cout << "总的工人匹配数为:" <<  worker_count.size() << endl;

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
                 double preference2 = task[taskid].Reward - (2 * detour_distance[taskid][j] * c); //偏好值为score
        if (preference2 > 0)
        {
          double preference = worker[j].score;            //偏好值为score
          PT[taskid].push_back(make_pair(j, preference)); //I任务的j worker对应的preference
                                                          //     cout<<taskid<<"\t"<<j<<"偏好计算成功"<<endl;
        }
          }
          else{
 //           cout<<taskid<<"\t"<<j<<"距离太远!"<<detour_distance[taskid][j]<<endl;
          }
      //    cout<<endl;
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

  void ReadLocationForTask(TASK (&task)[Number_Task]){
    ifstream in("F:\\CPP\\dataset\\G-mission\\task_information.txt");
  if (in){
  for (int i = 0; i < Number_Task; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{  
			double  temp;
		   	in >> temp;   
			if(j==2)
	          	 task[i].X = temp;
			if(j==3)
			       task[i].Y = temp;
			if(j==4)   //获取报酬
             task[i].Reward = temp;   
		}
	}
  
	in.close(); //关闭文件
  }
  /*  for(int i=0;i<Number_Task;i++)   //输出测试
	{
		printf("%.6f\t",task[i].X);
		printf("%.6f\n",task[i].Y);
	    // cout<<task[i].X<<"\t"<<task[i].Y<<endl;
	}
	*/

//读数据
/*	for (int i = 0; i < ROW; ++i)
	{
		for (int j = 0; j < VOL; ++j)
		{   
			if(j=0)
			in2 >> task[i].Reward;
			if(j=1)
			in2 >> task[i].Minscore;
		}
	}
	in2.close(); //关闭文件
	*/
}

void Get_Trajectory_locations(WORKER (&worker)[Number_Worker]){   
	//ifstream in1("F:\\CPP\\dataset\\G-mission\\worker_trajectory.txt"); 
  ifstream in1("F:\\CPP\\dataset\\G-mission\\200worker\\data.txt"); 
	for (int i = 0; i < Number_Worker; ++i)
	{
   for (int k =0 ; k < 5 ;k++){
       POI poi;
	     for (int j = 0; j < 4; ++j)
		   {  
			  double  temp;
		   	in1 >> temp;   
		  	if(j==2)
	        poi.X = temp;
			  if(j==3)
			   { 
            poi.Y = temp;
            worker[i].trajectory.push_back(poi);
         }
	   	}
	}
  }

   in1.close();

//ifstream in2("F:\\CPP\\dataset\\G-mission\\worker-score.txt"); 
ifstream in2("F:\\CPP\\dataset\\G-mission\\200worker\\score.txt"); 
	for (int i = 0; i < Number_Worker; ++i)
	{
   for (int k =0 ; k < 3; k++){
			  double  temp;
		   	in2 >> temp;   
		  	if(k==2)
	        worker[i].score = temp;		 
	}
  }
   in2.close();

/*for(int i=0;i <Number_Worker;i++)   //输出trajectory_location
{

  for(int j=0; j< worker[i].trajectory.size();j++)
  {
    cout<<"("<<worker[i].trajectory[j].X<<","<<worker[i].trajectory[j].Y<<")"<<" ";
  }
  cout<<"经过的bus站数量："<<worker[i].trajectory.size()<<endl;	
}
*/


}

  void Caculate_Sumdist_Trajectory(double (&Sumdis)[Number_Worker],WORKER (&worker)[Number_Worker]){
    for(int i=0; i < Number_Worker;i++){
	     double sum=0;
       for(int j=0; j < worker[i].trajectory.size()-1; j++){
      sum = sum+ GetDistance( worker[i].trajectory[j].Y, worker[i].trajectory[j].X ,worker[i].trajectory[j+1].Y, worker[i].trajectory[j+1].X );  	
	   }
	   Sumdis[i] = sum;
	//   cout<<i<<" "<<"all:" << Sumdis[i] <<endl;
	}

}


void Prodece_Task_Reward_Minscore(TASK (&task)[Number_Task]){    //随机生成task的reward和minscore
 default_random_engine e1,e2;
    // normal_distribution<double> u1(0.8,0.02);
     uniform_real_distribution<double> u1(0.6,0.9);  //score
	  // uniform_int_distribution<unsigned> u2(1,10);  //reward
     for(int i = 0; i < Number_Task; ++i){
            task[i].Minscore = u1(e1);
           // task[i].Reward = u2(e2);
        //    cout << "score:"<< task[i].Minscore << endl;
	   	//	cout <<"reward:"<< task[i].Reward << endl;
	 }
      
     
}

void Prodece_Worker_endTime_range_score(WORKER (&worker)[Number_Worker],double (&Sumdis)[Number_Worker],double endtimeX,double rangeX, double scoreX)
{  
	//设置参数endtime,range,score,Maxdetour;
   //  default_random_engine e;
    // uniform_real_distribution<double> u(60,100);  //score
     for(int i = 0; i < Number_Worker; ++i){
      worker[i].endTime = (Sumdis[i] / speed)  * (1 + endtimeX);
			worker[i].range = 1 * rangeX;
		//	worker[i].score = u(e)*scoreX;
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
  /* double radLat1 = rad(lat1);
   double radLng1 = rad(lng1);
   double radLat2 = rad(lat2);
   double radLng2 = rad(lng2);
   double a =abs(radLng1 - radLng2);
   double b =abs(radLat1 - radLat2);
   double h = pow(sin(b/2),2)+ cos(lat1) * cos(lat2) * pow(sin(a/2),2);
   double s = 2 * EARTH_RADIUS * sin(sqrt(h)) *1000 ;
*/

  double s = sqrt(pow((lat1-lat2),2) + pow((lng1 -lng2),2));   //欧氏距离
  return s;
}


bool cmp2( pair<int,double> a, pair<int,double> b){

return a.second< b.second;

}


double Caculate_Task_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PT)[Number_Task])
{
 double sum=0;
for(int i=0;i < Matching.size();i++){
    int workerid =Matching[i].second;
    int taskid =Matching[i].first ;
    int index = GetIndex_PT(workerid,taskid,PT);
    
    // if(index == PT[taskid].size())
   //  cout<<"未在任务中找到工人！"<<endl;
    double s=(PT[taskid][index].second - 0)/(PT[taskid][0].second -0);  //偏好值归一化,偏好列表最后一个偏好之为0
     sum = sum + s;
    //  cout<<taskid<<"\t"<<workerid<<"\t"<<index<<"\t"<<PT[taskid].size()<<"\t"<<s<<"\t"<< sum <<endl;
    //  cout<<endl;
  }
 return sum/Number_Task * 100;
}

double Caculate_Worker_Satisfaction(vector<pair<int, int>> (&Matching),vector<pair<int, double>> (&PW)[Number_Worker]){
vector<int>  CT_Worker[Number_Worker];
 for(int i=0;i<Matching.size();i++){  
   int workerid=Matching[i].second;
   int taskid =Matching[i].first;
   CT_Worker[workerid].push_back(taskid);
 }
/*
for(int i=0;i<Number_Worker;i++)
   { 
      cout<<i<<","<<CT_Worker[i].size()<<endl;
      cout<<endl;
   }
*/
 double allsum=0;
for(int i=0;i<Number_Worker;i++){
   if(CT_Worker[i].size() !=0 ){
     double sum=0,avg=0;
      for(int j=0; j< CT_Worker[i].size();j++){
      int taskid = CT_Worker[i][j];
      int index = GetIndex_PW(i,taskid,PW);
      double s =((PW[i][index].second - 0)/(PW[i][0].second -0)) ; //偏好值归一化
      sum=sum+s;
     // cout<<i<<","<<CT_Worker[i].size()<<endl;
  //   cout<<i<<"\t"<<taskid<<"\t"<<index<<"\t"<<PW[i].size()<<"\t"<<s<<"\t"<< sum <<endl;
   //  cout<<endl;
  }
   avg =sum/CT_Worker[i].size();
   allsum =allsum +avg;
  //cout<<avg<<endl;
  //cout<<allsum<<endl;
  }
 
}
 return allsum/Number_Worker * 100 ;

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
   {
   for(int j=1;j<Number_Worker;j++)
   {
    cout<<"偏移点"<<poi[i][j]<<endl;
    cout<<"任务"<<i<<",工人"<<j<<"的绕路距离:"<<detour_distance[i][j]<<endl;
    cout<<endl;
   }
   }
   
  /*
  double avaiable[Number_Worker];
for(int i=0;i<Number_Worker;i++)
{
   for(int j=0;j<Number_Task;j++)
   {
  if(detour_distance[i][j] < 1000)
      avaiable[i]++;
   }
}

int tongji[4]={0};
 for(int i=0;i<Number_Worker;i++)
{
    double min = avaiable[i];
   if(min > 500 && min <= 600 )
      tongji[0]++;
   if(min > 600 && min <= 700 )
      tongji[1]++;
       if(min > 700 && min <= 800 )
    tongji[2]++;
       if(min > 800 && min <= 900 )
    tongji[3]++;

}
    for(int i=0;i<4;i++)
        cout<<tongji[i]<<endl;
   cout<<endl;
*/

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
            if ( preference > maxpre)   //偏好值大于约束,********
            {
               double dist = Caculate_mindist(workerid,taskid,poi);
               if(worker[workerid].range >= dist)     //计算绕路距离距离小于range 
               {
                   if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束
                   {
                    double  preference2 = task[taskid].Reward - (2 * dist * c );  //T-Drive没加这个约束
                    if(preference2 >0)
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
#include "ImageProcess.h"

uint16_t RGBYUVImage[IMAGE_W][IMAGE_W] __attribute__((section(".ARM.__at_0x20000000")));
//uint8_t TowValueImage[IMAGE_W][IMAGE_W/8] __attribute__((section(".ARM.__at_0x1000d240")));
uint8_t GrayImage[IMAGE_W][IMAGE_W] __attribute__((section(".ARM.__at_0x10000000")));
point Centers[VOTENUMBER] __attribute__((section(".ARM.__at_0x1000d240")));
uint8_t RGB565toGary(uint16_t RGB565)
{
	uint8_t Gray = ((RGB565 >> 8) * 38 + ((RGB565 >> 3) & 0xff) * 75 + ((RGB565 << 3) & 0xff) * 15) >> 7;
	return Gray;
}
uint16_t Gray8toGary16(uint8_t Gray8)
{
	uint16_t Gary16 =((Gray8 >> 3 << 11) | (Gray8 >> 2 << 5) | (Gray8 >> 3));
	return Gary16;
}
void RGB_to_gray()
{
	for(uint16_t y=0;y<IMAGE_W;y++)
	{
		for(uint16_t x=0;x<IMAGE_W;x++)
		{
			uint8_t Gary=RGB565toGary(RGBYUVImage[y][x]);
			GrayImage[y][x]=Gary;
		}
	}
}
#define BN 4 //滤波数组大小
typedef struct filterstr
{
	float buffer[BN];
	float* pbuffer;
	bool FLAG_Begin;
}filterstr;
float filter(filterstr * filterpar,int value)
{
	static bool FLAG_Begin = false;
	*filterpar->pbuffer = value;
	if (filterpar->pbuffer == &filterpar->buffer[BN - 1])
	{
		filterpar->pbuffer = filterpar->buffer;
		FLAG_Begin = true;
	}
	else
		filterpar->pbuffer++;
	int Max = -1000, Min = 1000;
	float Sum = 0;
	if (FLAG_Begin)
	{
		for (uint8_t i = 0; i < BN; i++)
		{
			int Temp = filterpar->buffer[i];
			if (Max < Temp)
				Max = Temp;
			if (Min > Temp)
				Min = Temp;
			Sum += Temp;
		}
		Sum -= Min + Max;
		return (Sum / (BN - 2));
	}
	return 0;
}
Gradient Sobel_Gradient(uint8_t x,uint8_t y)
{
	Gradient G;
	uint16_t Up=	GrayImage[y-1][x-1]		+GrayImage[y-1][x]		+GrayImage[y-1][x]		+GrayImage[y-1][x+1];
	uint16_t Down=GrayImage[y+1][x-1]	+GrayImage[y+1][x]	+GrayImage[y+1][x]	+GrayImage[y+1][x+1];
	int Y=Up-Down;
	uint16_t Left=	GrayImage[y-1][x-1]	+GrayImage[y][x-1]	+GrayImage[y][x-1]	+GrayImage[y+1][x-1];
	uint16_t Right=	GrayImage[y-1][x+1]	+GrayImage[y][x+1]	+GrayImage[y][x+1]	+GrayImage[y+1][x+1];
	int X=Right-Left;
	G.lenth=__sqrtf(X*X+Y*Y);//4.8ms
	//G.lenth=_sqrtf(X*X+Y*Y);//4.8ms withoutFPU 10.7ms
	//G.lenth=sqrtf(X*X+Y*Y);//7.2ms
	//G.lenth=sqrt(X*X+Y*Y);//18ms
	//G.lenth=_sqrt(X*X+Y*Y);//18ms
	//G.lenth=(abs(Y)+abs(X));//3.2ms
	if(G.lenth>200)
	{
		G.X=X;
		G.Y=Y;
	}
	else
		G.lenth=0;
	return G;
}
uint8_t RADIUS=13,R_RANGE=3,HALF_R_RANGE=2;
void Vote(uint16_t x,uint16_t y,uint16_t * CentersNumber)
{
	Gradient G=Sobel_Gradient(x,y);
	if(G.lenth)//允许投票
	{
		/*计算投票点*/
		point tCenter[R_RANGE];
		if(abs(G.X)>abs(G.Y))
		{
			float K=(float)G.Y/G.X;
			float stepX=__sqrtf(1/(K*K+1));
			for(uint8_t i=0;i<R_RANGE;i++)
			{
					float r=RADIUS-HALF_R_RANGE+i;
					float steapX=stepX*r;
					float steapY=_fabsf(K*steapX);
					tCenter[i].x=G.X>0?x+steapX:x-steapX;
					tCenter[i].y=G.Y<0?y+steapY:y-steapY;
					tCenter[i].confidence=1;
			}
		}
		else
		{
			float K=(float)G.X/G.Y;
			float stepY=__sqrtf(1/(K*K+1));
			for(uint8_t i=0;i<R_RANGE;i++)
			{
				float r=RADIUS-HALF_R_RANGE+i;
				float steapY=stepY*r;
				float steapX=_fabsf(K*steapY);
				tCenter[i].x=G.X>0?x+steapX:x-steapX;
				tCenter[i].y=G.Y<0?y+steapY:y-steapY;
				tCenter[i].confidence=1;
			}
		}
		/*处理投票点*/
		uint8_t VoteNUM=R_RANGE;
		while(VoteNUM>0)
		{
			point ttCenter=tCenter[VoteNUM-1];
			bool EndVoteFLAG=false;
			for(uint16_t i=0;i<*CentersNumber;i++)//寻找是否是已投的点
			{
				if(Centers[i].x==ttCenter.x&&Centers[i].y==ttCenter.y)
				{
					Centers[i].confidence+=1;
					EndVoteFLAG=true;
					break;
				}
			}
			if(!EndVoteFLAG)//这个点是新点
			{
				if (*CentersNumber == VOTENUMBER)
					break;
				Centers[*CentersNumber]=ttCenter;
				(*CentersNumber)++;
			}
			VoteNUM--;
		}
	}
}
void ConvolutionAndVote(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t * CentersNumber)
{
	for (uint16_t y = y1; y < y2 && *CentersNumber < VOTENUMBER; y++)
	{
		for (uint16_t x = x1; x < x2 && *CentersNumber < VOTENUMBER; x++)
		{
			Vote(x, y, CentersNumber);
			x += 1;
		}
		y += 1;
	}
}
static filterstr POINTX={{0},POINTX.buffer,false};
static filterstr POINTY={{0},POINTY.buffer,false};
Center find_circle(uint8_t radius,uint8_t r_range,uint8_t Minconfidence)
{
	static Center Last_Center={0,0,0,0,0,0,0};
	RADIUS=radius,R_RANGE=r_range,HALF_R_RANGE=r_range/2;
	uint16_t CentersNumber = 0;
	ConvolutionAndVote(50,50,182,182,&CentersNumber);//中间矩形
	ConvolutionAndVote(1, 1, IMAGE_W-1, 50, &CentersNumber);//上边框
	ConvolutionAndVote(1, 182, IMAGE_W - 1, IMAGE_W - 1, &CentersNumber);//下边框
	ConvolutionAndVote(1, 50, 50, 182, &CentersNumber);//左边框
	ConvolutionAndVote(182, 50, IMAGE_W - 1, 182, &CentersNumber);//右边框
	Center max={0,0,0,0,0};
	if(CentersNumber)
	{
		for(uint16_t i=0;i<CentersNumber;i++)//寻找最大值的点
		{
			if(Centers[i].confidence>max.confidence)
			{
				max.x=Centers[i].x;
				max.y=Centers[i].y;
				max.confidence=Centers[i].confidence;
			}
		}
	}
	if(max.confidence<Minconfidence)
	{
		max=Last_Center;
	}
	max.CentersNumber=CentersNumber;
	max.Centers=Centers;
	max.x=filter(&POINTX,max.x);
	max.y=filter(&POINTY,max.y);
	max.speedx=max.x-Last_Center.x;
	max.speedy=max.y-Last_Center.y;
	Last_Center=max;
	return max;
}
#include "GM11_INT.h"
#include "myglobal.h"
#include "math.h"
void matrix_multiply(int *a,int *b,int *ans,int am,int an_bm,int bn)
{
	int li,lj,lkk;

	for(li=1;li<=am;li++)
	{
		for(lj=1;lj<=bn;lj++)
		{
			ans[(li-1)*bn+lj-1]=0;
			for(lkk=1;lkk<=an_bm;lkk++)
			{
				ans[(li-1)*bn+lj-1]+=a[(li-1)*an_bm+lkk-1]*b[(lkk-1)*bn+lj-1];
			}
		}
	}

}
void AGO(int *indata,int *outdata,int indatasize)
{
	int _i=0;
	outdata[0]=indata[0];
	for(_i=1;_i<indatasize;_i++)
	{
		outdata[_i]=outdata[_i-1]+indata[_i];
	}

}

//outdata[0]=z1(2)=a*x1[2]+(1-a)*x1[1]
void MEAN_AGO(int *indata,int* outdata,int indatasize,int para_100)
{
	int _i;
	for(_i=1;_i<indatasize;_i++)
	{
		outdata[_i]=(para_100*indata[_i]+(100-para_100)*indata[_i-1])/100;
	}
	outdata[0]=-1;
}
int Matrix_ni(int *mat_in,int *mat_out)
{
	int det=mat_in[3]*mat_in[0]-mat_in[1]*mat_in[2];

	mat_out[0]=mat_in[3];///det;
	mat_out[1]=-mat_in[1];///det;
	mat_out[2]=-mat_in[2];//det;
	mat_out[3]=mat_in[0];///det;
	
	return det;
}
void GM11_Save_Data(int data)
{
	int _i;
	for(_i=1;_i<gm11info.datalength;_i++)
	{
			gm11info.origin_data[_i-1]=gm11info.origin_data[_i];
	}
	gm11info.origin_data[gm11info.datalength-1]=data;
	if(gm11info.data_count<gm11info.datalength)
	{
		gm11info.data_count++;
		gm11info.data_full=0;
	 
	}
	else
	{
		gm11info.data_count=gm11info.datalength;
		gm11info.data_full=1;
	}
	
	
}
void GM11_Clear_AllData()
{	
	int _i;
	for(_i=0;_i<gm11info.datalength;_i++)
	{	
		gm11info.origin_data[_i]=0;
	}
	gm11info.data_count=0;
	gm11info.data_full=0;
	 
}
int GM11(void)
{

	int AGOdata[GM11_MAXDATALEN];
	int mean_AGOdata[GM11_MAXDATALEN];


	int B[GM11_MAXDATALEN-1][2];
	int BT[2][GM11_MAXDATALEN-1];
	int BTB[2][2];
	int BTBni[2][2];
	
	int BTBniBT[2][GM11_MAXDATALEN-1];
	int det_BTB;
	float P_f[2];//[a,b] 
	int P_I[2];//[a,b] 
	int _i;
	float predicted_AGO[2];

	int predicted_data;
	int temp1,temp2;
	
	float exp_nat;
	
	//int zxc[10];
	//int rrr=0;
	

	//matrix_multiply(&a[0][0],&b[0][0],&ans[0][0],2,3,3);
	AGO(gm11info.origin_data,AGOdata,gm11info.datalength);
	MEAN_AGO(AGOdata,mean_AGOdata,gm11info.datalength,50);
	for(_i=0;_i<gm11info.datalength-1;_i++)
	{
		B[_i][0]=-mean_AGOdata[_i+1];
		B[_i][1]=1;
		BT[0][_i]=-mean_AGOdata[_i+1];
		BT[1][_i]=1;
	}


	matrix_multiply(&BT[0][0],&B[0][0],&BTB[0][0],2,gm11info.datalength-1,2);

	det_BTB=Matrix_ni(&BTB[0][0],&BTBni[0][0]);

	if(det_BTB==0)
	{	
		gm11info.predict_fault=1;
	}
	matrix_multiply(&BTBni[0][0],&BT[0][0],&BTBniBT[0][0],2,2,gm11info.datalength-1);

	matrix_multiply(&BTBniBT[0][0],&gm11info.origin_data[1],P_I,2,gm11info.datalength-1,1);

	if(P_I[0]==0)
	{
			predicted_data=gm11info.origin_data[0];
			gm11info.predict_data=predicted_data;
	//	return predicted_data;
	}
	else
	{
			P_f[0]=(double)(P_I[0]);
			P_f[1]=(double)(P_I[1]);
			P_f[0]/=det_BTB;
			P_f[1]/=det_BTB;
			
			temp1=gm11info.origin_data[0]-P_I[1]/P_I[0];
			temp2=P_I[1]/P_I[0];
			for(_i=gm11info.datalength-1;_i<=gm11info.datalength;_i++)//new data is AGO
			{
				exp_nat=exp(-P_f[0]*_i);
				predicted_AGO[_i-(gm11info.datalength-1)]=temp1*exp_nat+temp2;
				 //newdata=(data[0]-P[1]/P[0])*(1-P[0]*r+(P[0]*P[0])/(r*r)-(P[0]*P[0]*P[0])/(r*r*r))+P[1]/P[0];
			 
			}

			predicted_data = predicted_AGO[1]-predicted_AGO[0];
			gm11info.predict_data=predicted_data;
			if(predicted_data==0)
			{
					error(9999);
			}
			
	}
	return predicted_data;
	
	
}

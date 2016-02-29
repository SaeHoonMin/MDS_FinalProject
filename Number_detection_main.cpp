#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/* g2450 application headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/kdev_t.h>

#include "g2450_pwm.h"
/****************************/


#define DEBUG




typedef int                 BOOL;
typedef struct
{
	BOOL	bVisitedFlag; //�湮����� ��Ÿ��
	CvPoint ptReturnPoint;//��������Ʈ
} Visited;


int period_arr[14] = {340000,320000,310000,300000,290000,280000,270000,260000,
	250000,240000,230000,220000,210000,200000};

	struct g2450_pwm_duty_t pwm_duty;

	int dev_fd;
	char dev_name[1024];
#define _DEF_MAX_BLOBS	10000		//�ִ�blob��

int			resolution = 200;
CvRect*		m_recBlobs;				// ���̺��� ���� (x, y, width, height)
int			m_nBlobs;				// ���̺��� ����
IplImage	*m_Image;				// ���̺��� ���� �̹���
int			m_nThreshold = 10;		// ���̺� ������Ȧ�� ��
Visited		*m_vPoint;				// ���̺��� �湮����
int			resultNum[8][2];		//�νĵ� ����� �� 
int			digit;					//�νĵ� ����� �ڸ����� ��Ÿ��
IplImage	*labelImage;			//���̺��� ǥ�õǴ� �̹��� ����

void CalcImage(IplImage* srcImage);
void Gray2Binary(IplImage *srcImage, IplImage *dstImage, int threshold);
void SetParam(IplImage* image, int nThreshold);
void DoLabeling();
int Labeling(IplImage* image, int nThreshold);
int _Labeling(unsigned char *DataBuf, int nWidth, int nHeight, int nThreshold);
void InitvPoint(int nWidth, int nHeight);
void DeletevPoint();
void DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight);
int __NRFIndNeighbor(unsigned char *DataBuf, int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY);
int __Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel);
CvRect GetNumImage(IplImage* srcImage);
void RecogNum(IplImage* srcImage);
int TemplateMatching(IplImage* image, IplImage* templ);
/*************************************************
* Capture an image from camera
**************************************************/
int main()
{

	/*		g2450_pwm init			*/



	bzero( dev_name, sizeof( dev_name ));
	sprintf( dev_name, "/dev/%s", DEV_PWM_NAME );
	mknod( dev_name, S_IFCHR|S_IRWXU|S_IRWXG, MKDEV( DEV_PWM_MAJOR, 0 ));
	printf("Make Device file(%s)\n", dev_name );

	dev_fd = open( dev_name, O_RDWR );
	if( 0 >= dev_fd )
	{
		printf("Open fail!\n");
		exit(0);
	}

	ioctl( dev_fd, DEV_PWM_STOP );
	pwm_duty.pulse_width= 100000;
	pwm_duty.period = 200000;
			//ioctl(dev_fd,DEV_PWM_RUN);
			//ioctl( dev_fd, DEV_PWM_DUTYRATE, &pwm_duty );

	//ioctl( dev_fd, DEV_PWM_RUN );

	/********************************************************************/


	IplImage *frame;
	std::cout<<"1"<<std::endl;
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	std::cout<<"2"<<std::endl;
	if(capture)
	{
		while (1)
		{
			cvGrabFrame(capture);
			frame = cvRetrieveFrame(capture);
		//	cvSaveImage("./1.frame.jpg", frame);
			CalcImage(frame);
		
		}
	}
	else
	{
		printf("CAM no detected!!");
	}
	cvReleaseCapture(&capture);
	return 0;
}
/*************************************************
* Image processing routine
* 1. check red circle within an captured image and copy
* 2. convert to a binary image by threshold value
* 3. labeling numbers
* 4. get the number area
* 5. compare selected number area and reference number image
* 6. define final number and print this
**************************************************/
void CalcImage(IplImage* srcImage)
{
	//IplImage *HSVImage = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 3);
	//IplImage *TempImage = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 1);
	//IplImage *TempGrayImage = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 1);
	IplImage *HSVImage = cvCreateImage(cvSize(srcImage->width,srcImage->height), IPL_DEPTH_8U, 3);
	IplImage *TempImage = cvCreateImage(cvSize(srcImage->width,srcImage->height), IPL_DEPTH_8U, 1);
	IplImage *TempGrayImage = cvCreateImage(cvSize(srcImage->width,srcImage->height), IPL_DEPTH_8U, 1);
	int i;
	// change RGB to HSV, calc circle position
	cvCvtColor(srcImage, HSVImage, CV_BGR2HSV);
	cvCvtColor(srcImage, TempGrayImage, CV_BGR2GRAY);
	cvInRangeS(HSVImage, cvScalar(80, 80, 80, 0), cvScalar(358, 256, 255, 0), TempImage);
	cvReleaseImage(&HSVImage);
	cvThreshold(TempImage, TempImage, 150, 100, CV_THRESH_BINARY);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* Circles = cvHoughCircles(TempImage, storage, CV_HOUGH_GRADIENT, 4, TempImage->width / 4, 200, 180, 25, 100);
	// end change RGB to HSV, calc circle position
    std::cout<<Circles->total<<std::endl;
	for (i = 0; i < Circles->total; i++)
	{
		float* circle;
		int cx, cy, radius, length;

		circle = (float*)cvGetSeqElem(Circles, i);
		cx = cvRound(circle[0]);
		cy = cvRound(circle[1]);
		radius = cvRound(circle[2]);
		length = radius * 2;

		// set ROI and copy
		cvSetImageROI(TempGrayImage, cvRect(cx - radius, cy - radius, length, length));
		IplImage* TempImage2 = cvCreateImage(cvSize(length, length), IPL_DEPTH_8U, 1);
		if (cx - radius <= 0 || cy - radius <= 0 || cx + radius >= TempImage->width || cy + radius >= TempImage->height)
		{
			cvReleaseImage(&TempImage2);
			cvResetImageROI(TempGrayImage);
			break;
		}
		else
		{
			cvCopy(TempGrayImage, TempImage2, 0);
		}
		cvResetImageROI(TempGrayImage);
		// end set ROI and copy
		IplConvKernel *element = cvCreateStructuringElementEx(7, 7, 3, 3, CV_SHAPE_RECT, NULL);

		//cvErode(TempImage2, TempImage2, element, 1);			//ħ�Ŀ���
		//cvDilate(TempImage2, TempImage2, element, 1);			//��â����
		//cvErode(TempImage2, TempImage2, element, 1);			//ħ�Ŀ���
		Gray2Binary(TempImage2, TempImage2, 60);				//����ȭ

		SetParam(TempImage2, resolution);
		DoLabeling();

		CvRect numRect = GetNumImage(TempImage2);//���̺� ���� �� ��ȣ���� ��������
		RecogNum(TempImage2);
		//		PrintResult();
		cvReleaseImage(&TempImage2);

	}
	cvClearSeq(Circles);
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&HSVImage);
	cvReleaseImage(&TempImage);
	cvReleaseImage(&TempGrayImage);


}

void Gray2Binary(IplImage *srcImage, IplImage *dstImage, int threshold)
{
	int nWidth = srcImage->width;
	int nHeight = srcImage->height;
	uchar *data = (uchar*)srcImage->imageData;

	for (int j = 0; j<srcImage->height; j++)
	{
		for (int i = 0; i < srcImage->width; i++)
		{
			//�Ӱ谪�̻��ΰ͸� ������� ����
			if (data[j*srcImage->widthStep + i]>threshold)
				dstImage->imageData[j*dstImage->widthStep + i] = 0;
			else
				dstImage->imageData[j*dstImage->widthStep + i] = 255;
		}
	}
}

void SetParam(IplImage* image, int nThreshold)
{
	int nWidth = image->width;
	int nHeight = image->height;

	if (m_recBlobs != NULL)
	{
		delete m_recBlobs;
		m_recBlobs = NULL;
		m_nBlobs = _DEF_MAX_BLOBS;
	}

	if (m_Image != NULL)
	{
		cvReleaseImage(&m_Image);
	}
	m_Image = cvCloneImage(image);

	m_nThreshold = nThreshold;
}

void DoLabeling()
{
	int Temp;
	m_nBlobs = Labeling(m_Image, m_nThreshold);

}

int Labeling(IplImage* image, int nThreshold)
{
	if (image->nChannels != 1) 	return 0;
	int nNumber;
	int nWidth = image->width;
	int nHeight = image->height;

	unsigned char* tmpBuf = new unsigned char[nWidth * nHeight];

	int i, j;
	for (j = 0; j<nHeight; j++)
		for (i = 0; i < nWidth; i++)
				tmpBuf[j*nWidth + i] = (unsigned char)image->imageData[j*image->widthStep + i];		// ����� ���� ������ ���۷� ����


	// ��, ����� ������ ���۴� ������ ����
	// ���̺��� ���� ����Ʈ �ʱ�ȭ
	InitvPoint(nWidth, nHeight);

	// ���̺�
	nNumber = _Labeling(tmpBuf, nWidth, nHeight, nThreshold);									// ������ ������ ���ϰ����� ����
	// ����Ʈ �޸� ����
	DeletevPoint();

	if (nNumber <= _DEF_MAX_BLOBS)		{
		m_recBlobs = new CvRect[nNumber];
	}
	if (nNumber > 0)	DetectLabelingRegion(nNumber, tmpBuf, nWidth, nHeight);

	for (j = 0; j<nHeight; j++)
		for (i = 0; i<nWidth; i++)
			image->imageData[j*image->widthStep + i] = tmpBuf[j*nWidth + i];

	delete tmpBuf;
	return nNumber;
}

int _Labeling(unsigned char *DataBuf, int nWidth, int nHeight, int nThreshold)
{
	int Index = 0, num = 0;
	int nX, nY, k, l;
	int StartX, StartY, EndX, EndY;
	// Find connected components

	for (nY = 0; nY < nHeight; nY++)
	{
		for (nX = 0; nX < nWidth; nX++)
		{
			if (DataBuf[nY * nWidth + nX] == 255)					// If a pixel value is white (255), increase one of num value
			{
				num++;
				DataBuf[nY * nWidth + nX] = (unsigned char)num;		// ���� ������ ��� ���ۿ� num�� ����, ���ο� ������ ã�� ��� num�� ���� 1 �������� ����

				StartX = nX, StartY = nY, EndX = nX, EndY = nY;		// ó�� ã�� �ȼ��� x �� y�� ��ġ�� ����, ���Ƿ� ������ ������ �κ��� ó�� ã�� �ȼ��� ��ġ�� �����ϰ� ����

				__NRFIndNeighbor(DataBuf, nWidth, nHeight, nX, nY, &StartX, &StartY, &EndX, &EndY);

				if (__Area(DataBuf, StartX, StartY, EndX, EndY, nWidth, num) < nThreshold)
				{
					for (k = StartY; k <= EndY; k++)
					{
						for (l = StartX; l <= EndX; l++)
						{
							if (DataBuf[k * nWidth + l] == (unsigned char)num)
								DataBuf[k * nWidth + l] = 0;
						}
					}
					--num;

					if (num > 250)
					return  0;
				}
			}
		}
	}
	return num;
}

void InitvPoint(int nWidth, int nHeight)
{
	int nX, nY;

	m_vPoint = new Visited[nWidth * nHeight];

	for (nY = 0; nY < nHeight; nY++)
	{
		for (nX = 0; nX < nWidth; nX++)
		{
			m_vPoint[nY * nWidth + nX].bVisitedFlag = FALSE;
			m_vPoint[nY * nWidth + nX].ptReturnPoint.x = nX;
			m_vPoint[nY * nWidth + nX].ptReturnPoint.y = nY;
		}
	}
}

void DeletevPoint()
{
	delete m_vPoint;
}

void DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight)
{
	int nX, nY;
	int nLabelIndex;
	char filename[1024];
	static int count = 0;
	BOOL bFirstFlag[255] = { FALSE, };
	for (nY = 1; nY < nHeight - 1; nY++)
	{
		for (nX = 1; nX < nWidth - 1; nX++)
		{
			nLabelIndex = DataBuf[nY * nWidth + nX];	// �󺧸� �� ���� nLabelIndex�� ����
			if (nLabelIndex > 0)	// Is this a new component?, 255 == Object
			{											// ���� �󺧸��� �Ǹ�(���� ������ �ش�Ǹ�) if�� ����


				if (bFirstFlag[nLabelIndex] == FALSE)	// �󺧸� ������ ó�� ���� �� ���
				{
					m_recBlobs[nLabelIndex - 1].x = nX;
					m_recBlobs[nLabelIndex - 1].y = nY;
					m_recBlobs[nLabelIndex - 1].width = 0;
					m_recBlobs[nLabelIndex - 1].height = 0;
					bFirstFlag[nLabelIndex] = TRUE;
				}
				else									// �󺧸� ������ ó�� �������� �ʾ��� ���
				{
					int left = m_recBlobs[nLabelIndex - 1].x;
					int right = left + m_recBlobs[nLabelIndex - 1].width;
					int top = m_recBlobs[nLabelIndex - 1].y;
					int bottom = top + m_recBlobs[nLabelIndex - 1].height;

					if (left >= nX)	left = nX;
					if (right <= nX)	right = nX;
					if (top >= nY)	top = nY;
					if (bottom <= nY)	bottom = nY;

					m_recBlobs[nLabelIndex - 1].x = left;
					m_recBlobs[nLabelIndex - 1].y = top;
					m_recBlobs[nLabelIndex - 1].width = right - left;
					m_recBlobs[nLabelIndex - 1].height = bottom - top;
				}
			}
		}
	}
}

int __NRFIndNeighbor(unsigned char *DataBuf, int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY)
{
	CvPoint CurrentPoint;

	CurrentPoint.x = nPosX;
	CurrentPoint.y = nPosY;

	m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].bVisitedFlag = TRUE;
	m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.x = nPosX;
	m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.y = nPosY;

	while (1)
	{
		if ((CurrentPoint.x != 0) && (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x - 1] == 255))   // -X ����
		{
			if (m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x - 1].bVisitedFlag == FALSE)
			{
				DataBuf[CurrentPoint.y  * nWidth + CurrentPoint.x - 1] = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
				m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x - 1].bVisitedFlag = TRUE;
				m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x - 1].ptReturnPoint = CurrentPoint;
				CurrentPoint.x--;

				if (CurrentPoint.x <= 0)
					CurrentPoint.x = 0;

				if (*StartX >= CurrentPoint.x)
					*StartX = CurrentPoint.x;

				continue;
			}
		}

		if ((CurrentPoint.x != nWidth - 1) && (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x + 1] == 255))   // +X ����
		{
			if (m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x + 1].bVisitedFlag == FALSE)
			{
				DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x + 1] = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
				m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x + 1].bVisitedFlag = TRUE;
				m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x + 1].ptReturnPoint = CurrentPoint;
				CurrentPoint.x++;

				if (CurrentPoint.x >= nWidth - 1)
					CurrentPoint.x = nWidth - 1;

				if (*EndX <= CurrentPoint.x)
					*EndX = CurrentPoint.x;

				continue;
			}
		}

		if ((CurrentPoint.y != 0) && (DataBuf[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x] == 255))   // -X ����
		{
			if (m_vPoint[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x].bVisitedFlag == FALSE)
			{
				DataBuf[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x] = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
				m_vPoint[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x].bVisitedFlag = TRUE;
				m_vPoint[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x].ptReturnPoint = CurrentPoint;
				CurrentPoint.y--;

				if (CurrentPoint.y <= 0)
					CurrentPoint.y = 0;

				if (*StartY >= CurrentPoint.y)
					*StartY = CurrentPoint.y;

				continue;
			}
		}

		if ((CurrentPoint.y != nHeight - 1) && (DataBuf[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x] == 255))   // -X ����
		{
			if (m_vPoint[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x].bVisitedFlag == FALSE)
			{
				DataBuf[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x] = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
				m_vPoint[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x].bVisitedFlag = TRUE;
				m_vPoint[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x].ptReturnPoint = CurrentPoint;
				CurrentPoint.y++;

				if (CurrentPoint.y >= nHeight - 1)
					CurrentPoint.y = nHeight - 1;

				if (*EndY <= CurrentPoint.y)
					*EndY = CurrentPoint.y;

				continue;
			}
		}

		if ((CurrentPoint.x == m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.x)
			&& (CurrentPoint.y == m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.y))
		{
			break;
		}
		else
		{
			CurrentPoint = m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint;
		}
	}
	return 0;
}

int __Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel)
{
	int nArea = 0;
	int nX, nY;

	for (nY = StartY; nY < EndY; nY++)
		for (nX = StartX; nX < EndX; nX++)
			if (DataBuf[nY * nWidth + nX] == nLevel)
				++nArea;

	return nArea;
}

CvRect GetNumImage(IplImage* srcImage)
{
	//���� ���� ����
	int nWidth = srcImage->widthStep;
	int nHeight = srcImage->height;
	int nBlob = 0;

	//���̺� �� ������� ��ȣ������ �����Ѵ�.
	for (int i = 0; i < m_nBlobs; i++)
	{
		//��ȣ ������ ǥ���� ����
		CvPoint pt1 = cvPoint(m_recBlobs[i].x, m_recBlobs[i].y);
		CvPoint pt2 = cvPoint(pt1.x + m_recBlobs[i].width, pt1.y + m_recBlobs[i].height);
		// �̹��� ���ɿ��� ����
		if (m_recBlobs[i].x < 0 ||
			m_recBlobs[i].y < 0 ||
			m_recBlobs[i].width < 0 ||
			m_recBlobs[i].height < 0 ||
			m_recBlobs[i].x > srcImage->width ||
			m_recBlobs[i].x > srcImage->width)	continue;

		cvSetImageROI(srcImage, m_recBlobs[i]);
		IplImage* sub_gray = cvCreateImage(cvSize(m_recBlobs[i].width, m_recBlobs[i].height), 8, 1);

		//cvThreshold(srcImage, sub_gray, 1, 255, CV_THRESH_BINARY_INV);

		// ���ɿ��� ����
		cvResetImageROI(srcImage);

		////////////////////////////
		// ���̺�
		SetParam(sub_gray, resolution);
		DoLabeling();

		for (int j = 0; j < m_nBlobs; j++)
		{
			//CvPoint	s_pt1 = cvPoint(pt1.x + m_recBlobs[j].x, pt1.y + m_recBlobs[j].y);
			//CvPoint s_pt2 = cvPoint(s_pt1.x + m_recBlobs[j].width, s_pt1.y + m_recBlobs[j].height);
			CvPoint	s_pt1 = cvPoint(m_recBlobs[j].x, m_recBlobs[j].y);
			CvPoint s_pt2 = cvPoint(s_pt1.x + m_recBlobs[j].width, s_pt1.y + m_recBlobs[j].height);
		}
		cvReleaseImage(&sub_gray);
	}

	cvSaveImage("./test2.jpg", srcImage);

	return m_recBlobs[nBlob];
}

void RecogNum(IplImage* srcImage)
{
	static int temp, velocity=0;
	static int count=0;

	IplImage* numImage[10];//0~9���� ��ȣ�� ������ �̹�������
	char pathName[128];//�̹����� ������ ���
	digit = 0; //�νĵ� ����� �ڸ����� 0���� �ʱ�ȭ
	//labelImage = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 3);
	labelImage = cvCreateImage(cvSize(srcImage->width,srcImage->height), IPL_DEPTH_8U, 3);
	cvCvtColor(srcImage, labelImage, CV_GRAY2BGR);//�׷��̸� RGB�� �ٲ��ش�.

	FILE* fp = fopen("./matching.txt", "w+");
	char buf[512];

	//���ڿ��� ����
	SetParam(srcImage, resolution);
	DoLabeling();

	//�����ĺ������� ���̺� ����� ���ø� ��Ī
	for (int i = 0; i<m_nBlobs; i++)
	{
		CvRect rect = m_recBlobs[i];
		//������ ���̰� ���κ��� ��͸� ���ʸ���Ī�� �õ�
		if (rect.height<rect.width ||
		(float)rect.height / (float)rect.width>5 ||
		(float)rect.height / (float)rect.width < 1.3){

		}
		else
		{
			CvPoint pt1 = cvPoint(m_recBlobs[i].x, m_recBlobs[i].y);
			CvPoint pt2 = cvPoint(pt1.x + m_recBlobs[i].width, pt1.y + m_recBlobs[i].height);
			CvScalar color = cvScalar(0, 0, 255);
			cvDrawRect(labelImage, pt1, pt2, color);

			//���̺� ũ�⿡ ���߾� ���ڹ�ȣ �̹������� �����´�
			for (int n = 0; n<10; n++)
			{
				numImage[n] = cvCreateImage(cvSize(rect.width, rect.height), IPL_DEPTH_8U, 1);
				sprintf(pathName, "./num/%d.jpg", n);
				IplImage* temp = cvLoadImage(pathName, 0);
				cvResize(temp, numImage[n], CV_INTER_LINEAR);
				cvThreshold(numImage[n], numImage[n], 1, 255, CV_THRESH_BINARY_INV);
			}
			//���� blob�̹����� blobImage�� ����
			IplImage* blobImage = cvCreateImage(cvSize(rect.width, rect.height), IPL_DEPTH_8U, 1);

			cvSetImageROI(srcImage, rect);//��������
			cvCopy(srcImage, blobImage, NULL);//��������
			cvResetImageROI(srcImage);//��������

			cvThreshold(blobImage, blobImage, 1, 255, CV_THRESH_BINARY_INV);

			//���ø���Ī ����
			float max = 0;
			int matchingNum = 0;
			for (int n = 0; n < 10; n++)
			{
				int matchingCount = TemplateMatching(blobImage, numImage[n]);
				float matchingPercent = 100 * (float)matchingCount / (float)(blobImage->width*blobImage->height);
				sprintf(buf, "%d��° blob�� ���� %d�� ����blob�� �ۼ�Ʈ : %.2f\n", i, n, matchingPercent);
				fputs(buf, fp);

				if (matchingPercent > max)
				{
					max = matchingPercent;
					matchingNum = n;
				}
			}
			fputs("\n", fp);

			//��Ī���� 50%�̻��̸� �ش� ���ڸ� ����
			if (max > 35.0f)
			{
				resultNum[digit][0] = rect.x;
				resultNum[digit][1] = matchingNum;
				digit++;
			}
			cvReleaseImage(&blobImage);
		}
	}

	// 2�ڸ� ������ ���
	if (digit == 2)
	{

		if (resultNum[0][1]!=0 && (resultNum[0][1] * 10 + resultNum[1][1]) % 5 == 0)
			temp=resultNum[0][1] * 10 + resultNum[1][1];
		else if (resultNum[1][1] != 0 && (resultNum[1][1] * 10 + resultNum[0][1]) % 5 == 0)
			temp=resultNum[1][1] * 10 + resultNum[0][1];

#ifdef DEBUG
		std::cout<<"match found\n";
		std::cout<<"temp : "<<temp<<std::endl;
#endif
	}

	// 3�ڸ� ������ ���
	// ���� �ڸ� ���ڴ� �ݵ�� 1
	else if (digit == 3)
	{
		if (resultNum[0][1] == 1)
			if (resultNum[1][1] != 0 && (resultNum[1][1] * 10 + resultNum[2][1]) % 5 == 0)
				temp=resultNum[0][1] * 100 + resultNum[1][1] * 10 + resultNum[2][1];
			else if (resultNum[2][1] != 0 && (resultNum[2][1] * 10 + resultNum[1][1]) % 5 == 0)
				temp=resultNum[0][1] * 100 + resultNum[2][1] * 10 + resultNum[1][1];
			else
				temp=resultNum[0][1] * 100 + resultNum[1][1] * 10 + resultNum[2][1];
		else if (resultNum[1][1] == 1)
			if (resultNum[0][1] != 0 && (resultNum[0][1] * 10 + resultNum[2][1]) % 5 == 0)
				temp=resultNum[1][1] * 100 + resultNum[0][1] * 10 + resultNum[2][1];
			else if (resultNum[2][1] != 0 && (resultNum[2][1] * 10 + resultNum[0][1]) % 5 == 0)
				temp=resultNum[1][1] * 100 + resultNum[2][1] * 10 + resultNum[0][1];
			else
				temp=resultNum[1][1] * 100 + resultNum[0][1] * 10 + resultNum[2][1];
		else if (resultNum[2][1] == 1)
			if (resultNum[1][1] != 0 && (resultNum[1][1] * 10 + resultNum[0][1]) % 5 == 0)
				temp=resultNum[2][1] * 100 + resultNum[1][1] * 10 + resultNum[0][1];
			else if (resultNum[0][1] != 0 && (resultNum[0][1] * 10 + resultNum[1][1]) % 5 == 0)
				temp=resultNum[2][1] * 100 + resultNum[0][1] * 10 + resultNum[1][1];
			else
				temp=resultNum[2][1] * 100 + resultNum[0][1] * 10 + resultNum[1][1];
	}

	if(temp==velocity)
	{
		count++;
#ifdef DEBUG
		std::cout<<"count : "<<count<<std::endl;
#endif
		if(count>=3)
		{
			count=0;
			/*velocity*/
#ifdef DEBUG
		std::cout<<"velocity/10 : "<<velocity/10<<std::endl;
#endif
			pwm_duty.period = period_arr[velocity/10];
			ioctl(dev_fd,DEV_PWM_RUN);
			ioctl( dev_fd, DEV_PWM_DUTYRATE, &pwm_duty );
		}
	}
	else
	{
		std::cout<<"temp : "<<temp<<"velocity : "<<velocity<<std::endl;
		velocity=temp;
		count=0;
	}

	fclose(fp);
}

int TemplateMatching(IplImage* image, IplImage* templ)
{
	int percent = 0;
	int nWidth = image->widthStep;
	int nHeight = image->height;

	for (int i = 0; i<nHeight; i++)
	{
		for (int j = 0; j<nWidth; j++)
		{
			if (image->imageData[i*nWidth + j] == templ->imageData[i*nWidth + j])
				percent++;
		}
	}

	return percent;
}

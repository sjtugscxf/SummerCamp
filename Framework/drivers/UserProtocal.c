#include "UserProtocal.h"


//�淶��ʽ��֡ͷ+����+�ָ���+����+�ָ���+...+֡β������ֵ�������������ݵ��ַ���ά����
//����ֵ��0ʧ�ܣ���dataout��ά��
u8 ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[][15])
{
    u8 headlength,endlength,datalength,totallength;
    u8 i=0;//�ָ������
    char temp[50]="";//���������ַ�������
    char*splitchar;//�ָ���
    headlength=strlen(head);
    endlength=strlen(end);
    totallength=strlen(rxbuf);
		datalength=totallength-headlength-endlength;
    strncpy(temp,rxbuf,headlength);
    temp[headlength]='\0';
    if(strcmp(temp,head))
    {
        return 0;
    }
    strncpy(temp,rxbuf+totallength-endlength,endlength);
    temp[endlength]='\0';
    if(strcmp(temp,end))
    {
        return 0;
    }
    strncpy(temp,rxbuf+headlength,datalength);
    temp[datalength]='\0';

    splitchar=strtok((char*)temp,separater);
    while(splitchar!=NULL)
    {
        sprintf(dataout[i++],"%s",splitchar);
        splitchar=strtok(NULL,separater);
    }
    return i;
}


/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
@ingroup icub_tools

\defgroup dataDumper dataDumper
 
Acquires and stores Bottles or Images and Videos from a YARP 
port. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 
 
Date: first release 16/05/2008 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

When launched, the service monitors the presence of incoming 
data (bottles or images) and stores it within a folder called 
with the same name as the service port (or with a proper suffix 
appended if other data is present in the current path). In this 
folder the file 'data.log' contains the information (taken from 
the envelope field of the port) line by line as follows: 
 
\code 
[pck id] [time stamp] [bottle content (or image_file_name)] 
   0       1.234         0 1 2 3 ...
   1       1.235         4 5 6 7 ...
   ...     ...           ...
\endcode 
 
Note that if the envelope is not valid, then the Time Stamp is 
the reference time of the machine where the service is running. 
Anyway, a selection between these two Time Stamps is available 
for the user through --rxTime option. 
 
Moreover, a further file called 'info.log' is also produced 
containing information about the data type stored within the 
'data.log' file as well as the name of the yarp ports connected 
or disconnected to the dumper, as in the following:

\code 
Type: [Bottle | Image:ppm | Image:ppm; Video:avi(huffyuv)]
[local-timestamp] /yarp-port-name [connected] 
[local-timestamp] /yarp-port-name [disconnected]
\endcode 
 
The module \ref dataSetPlayer can be used to re-play a dump generated 
by the \ref dataDumper.
 
\section lib_sec Libraries 
- YARP libraries 
- To record videos: OpenCV 2.0 and the <a 
  href="http://neuron2.net/www.math.berkeley.edu/benrg/huffyuv.html">huffyuv</a>
  codec for lossless data compression.
 
\section codec_installation Codec Installation 
- Windows: fetch the dll library from the codec website and 
  simply install it.
 
- Linux: ffmpeg must be properly installed and recognized by 
  OpenCV. The command "sudo apt-get ffmpeg" seemed not to work.
  This is a possible alterative procedure:
 
  -# sudo apt-get install libjpeg62-dev libtiff4-dev
     libjasper-dev libopenexr-dev libeigen2-dev yasm libfaac-dev
     libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
     libvorbis-dev libxvidcore-dev
  -# svn checkout svn://svn.ffmpeg.org/ffmpeg/trunk ffmpeg
  -# ./configure --enable-gpl --enable-version3 --enable-nonfree
     --enable-postproc --enable-libfaac --enable-libopencore-amrnb
     --enable-libopencore-amrwb --enable-libtheora
     --enable-libvorbis --enable-libxvid --enable-x11grab
     --enable-swscale --enable-shared
  -# sudo make install
  -# recompile OpenCV through cmake

\section parameters_sec Parameters
--name \e portname 
- The parameter \e portname identifies the name of the listening
  port open by the service; if not specified \e /dump is
  assumed. The leading forward slash will be added if not
  provided.
 
--dir \e dirname 
- The parameter \e dirname serves to specify the storage directory
  explicitely.
 
--overwrite 
- If this option is specified, then a pre-existing storage 
  directory will be overwritten.
 
--type \e datatype 
- The parameter \e datatype selects the type of items to be 
  stored. It can be \e bottle, \e image or \e video; if not
  specified \e bottle is assumed. Note that images are stored as
  ppm files. The data type \e video is available if OpenCV is
  found and the codec \e huffyuv is installed.
 
--addVideo
- In case images are acquired with this option enabled, a video 
  called 'video.avi' is also produced at the same time. This
  option is available if OpenCV is found and the codec \e
  huffyuv is installed in the system.
 
--downsample \e n 
- With this option it is possible to reduce the storing rate by 
  a factor \e n, i.e. the parameter \e n specifies how many
  items (bottles or images) shall be skipped after one
  acquisition.

--rxTime
- With this option it is possible to select which Time Stamp to
  assign to the dumped data: normally the sender time is the
  reference, but if this option is enabled, the time of the
  receiving machine on which the service is running will be the
  reference. However, even if --rxTime is not selected, the
  receiver time will be taken as reference in case of invalid
  message envelope.
 
--txTime
- Regardless of its availability, the sender time stamp will be
  inserted in the log straightaway. Moreover, if \e txTime is
  given in conjunction with the \e rxTime option (or even if
  \e rxTime is not selected but the sender stamp is invalid),
    then the following format will be adopted for the logged
    data:
 
\code 
[pck id] [tx stamp] [rx stamp] [bottle content]
\endcode 
 
\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <portname> (e.g. /dump) 
 
- \e <portname>/rpc which is a remote procedure call port useful
  to shut down the service remotely by sending to this port the
  'quit' command.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
Within the directory \e ./<portname> the file \e data.log is 
created containing the acquisitions report. Beside this, if \e 
image type has been selected, all the acquired images are also 
stored. 
 
Note that in case an acquisition with the same \e <portname> was
previously carried out, an increasing suffix will be appended 
to the name of the directory.
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux and Windows. 
 
\section example_sec Example
By launching the following command: 
 
\code 
dataDumper --name /log --type image --downsample 2 
\endcode 
 
the service will create the listening port /log capable of 
storing images in ppm format within the subdirectory ./log at a 
halved rate with respect to the rate of the sender. 
 
Then, by connecting the port to the sender with the usual yarp 
command
 
\code 
yarp connect /grabber /log 
\endcode 
 
the acquisition will start. 
 
By pressing CTRL+C the acquisition is terminated.
 
So, now, have a look inside the directory ./log 

\author Ugo Pattacini 

\sa dataSetPlayer
*/ 

#include <stdio.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <deque>

#ifdef ADD_VIDEO
    #include <cv.h>
    #include <highgui.h>
#endif

#include <yarp/os/all.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/**************************************************************************/
void createFullPath(const char* path)
{
    if (yarp::os::stat(path))
    {
        string strPath=string(path);
        size_t found=strPath.find_last_of("/");
    
        while (strPath[found]=='/')
            found--;

        createFullPath(strPath.substr(0,found+1).c_str());
        yarp::os::mkdir(strPath.c_str());
    }
}


/**************************************************************************/
typedef enum { bottle, image } DumpType;


// Abstract object definition for queueing
/**************************************************************************/
class DumpObj
{
public:
    virtual ~DumpObj() { }
    virtual const string toFile(const char*, unsigned int) = 0;
    virtual void *getPtr() = 0;
};


// Specialization for Bottle object
/**************************************************************************/
class DumpBottle : public DumpObj
{
private:
    Bottle *p;

public:
    DumpBottle() { p=new Bottle; }
    DumpBottle(const DumpBottle &obj) { p=new Bottle(*(obj.p)); }
    DumpBottle(const Bottle &b) { p=new Bottle(b); }
    const DumpBottle &operator=(const DumpBottle &obj) { *p=*(obj.p); return *this; }
    ~DumpBottle() { delete p; }

    const string toFile(const char *dirName, unsigned int cnt)
    {
        string ret=p->toString().c_str();
        return ret;
    }

    void *getPtr() { return NULL; }
};


// Object creator of type Bottle - overloaded
/**************************************************************************/
DumpObj *factory(Bottle& obj)
{
    DumpBottle *p=new DumpBottle(obj);
    return p;
}


// Specialization for Image object
/**************************************************************************/
class DumpImage : public DumpObj
{
private:
    ImageOf<PixelBgr> *p;

public:
    DumpImage() { p=new ImageOf<PixelBgr>(); }
    DumpImage(const DumpImage &obj) { p=new ImageOf<PixelBgr>(*(obj.p)); }
    DumpImage(const ImageOf<PixelBgr> &img) { p=new ImageOf<PixelBgr>(img); }
    const DumpImage &operator=(const DumpImage &obj) { *p=*(obj.p); return *this; }
    ~DumpImage() { delete p; }

    const string toFile(const char *dirName, unsigned int cnt)
    {
        char extfName[255];
        char fName[255];

        sprintf(fName,"%.8d.ppm",cnt);
        string ret=fName;

        sprintf(extfName,"%s/%s",dirName,fName);
        file::write(*p,extfName);

        return ret;
    }

    void *getPtr() { return p->getIplImage(); }
};


// Object creator of type Image - overloaded
/**************************************************************************/
DumpObj *factory(ImageOf<PixelBgr> &obj)
{
    DumpImage *p=new DumpImage(obj);
    return p;
}


// Class to manage tx and rx time stamps
/**************************************************************************/
class DumpTimeStamp
{
    double rxStamp;
    double txStamp;
    bool   rxOk;
    bool   txOk;

public:
    DumpTimeStamp() : rxOk(false), txOk(false) { }
    void setRxStamp(const double stamp) { rxStamp=stamp; rxOk=true; }
    void setTxStamp(const double stamp) { txStamp=stamp; txOk=true; }
    double getStamp() const
    {
        if (txOk)
            return txStamp;
        else if (rxOk)
            return rxStamp;
        else
            return -1.0;
    }
    string getString() const
    {
        ostringstream ret;
        ret<<fixed;

        if (txOk)
            ret<<txStamp;
        if (rxOk)
        {
            if (!ret.str().empty())
                ret<<' ';
            ret<<rxStamp; 
        }
        return ret.str();
    }
};


// Definition of item to be put in the queue
/**************************************************************************/
typedef struct
{
    int            seqNumber;
    DumpTimeStamp  timeStamp;
    DumpObj       *obj;
} DumpItem;


// Definition of the queue
// Two services act on this resource:
// 1) the port, which listens to incoming data
// 2) the thread, which stores the data to disk
/**************************************************************************/
class DumpQueue : public deque<DumpItem>
{
private:
    Mutex mutex;

public:
    void lock()   { mutex.lock();   }
    void unlock() { mutex.unlock(); }
};


/**************************************************************************/
template <class T>
class DumpPort : public BufferedPort<T>
{
public:
    DumpPort(DumpQueue &Q, unsigned int _dwnsample=1, bool _rxTime=true, bool _txTime=false) : buf(Q)
    {
        rxTime=_rxTime;
        txTime=_txTime;
        dwnsample=_dwnsample>0?_dwnsample:1;
        cnt=0;

        firstIncomingData=true;
    }

private:
    DumpQueue &buf;
    unsigned int dwnsample;
    unsigned int cnt;
    bool firstIncomingData;
    bool rxTime;
    bool txTime;

    void onRead(T &obj)
    {
        if (++cnt==dwnsample)
        {
            if (firstIncomingData)
            {
                cout << "Incoming data detected" << endl;
                firstIncomingData=false;
            }

            DumpItem item;
            Stamp info;

            BufferedPort<T>::getEnvelope(info);
            item.seqNumber=info.getCount();

            if (txTime || (info.isValid() && !rxTime))
                item.timeStamp.setTxStamp(info.getTime());

            if (rxTime || !info.isValid())
                item.timeStamp.setRxStamp(Time::now());

            item.obj=factory(obj);

            buf.lock();
            buf.push_back(item);
            buf.unlock();

            cnt=0;
        }
    }
};


/**************************************************************************/
class DumpThread : public RateThread
{
private:
    DumpQueue     &buf;
    DumpType       type;
    ofstream       finfo;
    ofstream       fdata;
    char           dirName[255];
    char           infoFile[255];
    char           dataFile[255];
    unsigned int   blockSize;
    unsigned int   cumulSize;
    unsigned int   counter;
    double         oldTime;
    
    bool           saveData;
    bool           videoOn;
    bool           closing;

#ifdef ADD_VIDEO
    char           videoFile[255];    
    bool           doImgParamsExtraction;
    bool           doSaveFrame;
    CvVideoWriter *videoWriter;    
#endif

public:
    DumpThread(DumpType _type, DumpQueue &Q, const char *_dirName, int szToWrite, bool _saveData, bool _videoOn) :
               RateThread(50), type(_type), buf(Q), blockSize(szToWrite), saveData(_saveData), videoOn(_videoOn)
    {
        strcpy(dirName,_dirName);

        strcpy(infoFile,dirName);
        strcat(infoFile,"/info.log");

        strcpy(dataFile,dirName);
        strcat(dataFile,"/data.log");

    #ifdef ADD_VIDEO
        strcpy(videoFile,dirName);
        strcat(videoFile,"/video.avi");

        doImgParamsExtraction=videoOn;
        doSaveFrame=false;
    #endif
    }

    void writeSource(const string &sourceName, const bool connected)
    {
        finfo << "[" << fixed << Time::now() << "] ";
        finfo << sourceName << " ";
        finfo << (connected?"[connected]":"[disconnected]") << endl;
    }

    bool threadInit()
    {
        oldTime=Time::now();
        cumulSize=0;
        counter=0;
        closing=false;

        finfo.open(infoFile);
        if (!finfo.is_open())
        {
            cout << "unable to open file" << endl;
            return false;
        }

        fdata.open(dataFile);
        if (!fdata.is_open())
        {
            cout << "unable to open file" << endl;
            return false;
        }

        finfo<<"Type: ";
        if (type==bottle)
            finfo<<"Bottle;";
        else if (type==image)
        {
            finfo<<"Image:ppm;";
            if (videoOn)
                finfo<<" Video:avi(huffyuv);";
        }

        finfo<<endl;
        return true;
    }

    void run()
    {
        buf.lock();
        unsigned int sz=buf.size(); //!!! access to size must be protected: problem spotted with Linux stl
        buf.unlock();

        // each 10 seconds it issues a writeToDisk command straightaway
        bool writeToDisk=false;
        double curTime=Time::now();
        if ((curTime-oldTime>10.0) || closing)
        {
            writeToDisk=sz>0;
            oldTime=curTime;
        }

        // it performs the writeToDisk on command or as soon as
        // the queue size is greater than the given threshold
        if ((sz>blockSize) || writeToDisk)
        {
        #ifdef ADD_VIDEO
            // extract images parameters just once
            if (doImgParamsExtraction && (sz>1))
            {
                buf.lock();
                DumpItem itemFront=buf.front();
                DumpItem itemEnd=buf.back();
                buf.unlock();

                int fps;
                int frameW=((IplImage*)itemEnd.obj->getPtr())->width;
                int frameH=((IplImage*)itemEnd.obj->getPtr())->height;

                double dt=itemEnd.timeStamp.getStamp()-itemFront.timeStamp.getStamp();
                if (dt<=0.0)
                    fps=25; // default
                else
                    fps=int(double(sz-1)/dt);

                videoWriter=cvCreateVideoWriter(videoFile,CV_FOURCC('H','F','Y','U'),
                                                fps,cvSize(frameW,frameH),true);

                doImgParamsExtraction=false;
                doSaveFrame=true;
            }
        #endif

            // save to disk
            for (unsigned int i=0; i<sz; i++)
            {
                buf.lock();
                DumpItem item=buf.front();
                buf.pop_front();
                buf.unlock();

                fdata << item.seqNumber << ' ' << item.timeStamp.getString() << ' ';
                if (saveData)
                    fdata << item.obj->toFile(dirName,counter++) << endl;
                else
                {
                    char frame[255];
                    sprintf(frame,"frame_%.8d",counter++);
                    fdata << frame << endl;
                }

            #ifdef ADD_VIDEO
                if (doSaveFrame)
                    cvWriteFrame(videoWriter,(IplImage*)item.obj->getPtr());
            #endif

                delete item.obj;
            }

            cumulSize+=sz;
            cout << sz << " items stored [cumul #: " << cumulSize << "]" << endl;
        }
    }

    void threadRelease()
    {
        // call run() for the last time to flush the queue
        closing=true;
        run();

        finfo.close();
        fdata.close();

    #ifdef ADD_VIDEO
        if (doSaveFrame)
            cvReleaseVideoWriter(&videoWriter);
    #endif
    }
};


/**************************************************************************/
class DumpReporter : public PortReport
{
private:
    DumpThread *thread;

public:
    DumpReporter() : thread(NULL) { }
    void setThread(DumpThread *thread) { this->thread=thread; }
    void report(const PortInfo &info)
    {
        if ((thread!=NULL) && info.incoming)
            thread->writeSource(info.sourceName.c_str(),info.created);
    }
};


/**************************************************************************/
class DumpModule: public RFModule
{
private:
    DumpQueue                    *q;
    DumpPort<Bottle>             *p_bottle;
    DumpPort<ImageOf<PixelBgr> > *p_image;
    DumpThread                   *t;
    DumpReporter                  reporter;
    Port                          rpcPort;
    DumpType                      type;
    bool                          saveData;
    bool                          videoOn;
    bool                          rxTime;
    bool                          txTime;
    unsigned int                  dwnsample;
    char                          portName[255];

public:
    DumpModule() { }

    bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        if (rf.check("name"))
        {
            strcpy(portName,rf.find("name").asString().c_str());
            if (portName[0]!='/')
            {
                string slashedPortName='/'+string(portName);
                strcpy(portName,slashedPortName.c_str());
            }
        }
        else
            strcpy(portName,"/dump");

        saveData=true;
        videoOn=false;

        if (rf.check("type"))
        {
            string optTypeName;
            optTypeName=rf.find("type").asString().c_str();

            if (optTypeName=="bottle")
                type=bottle;
            else if (optTypeName=="image")
            {    
                type=image;
            #ifdef ADD_VIDEO
                if (rf.check("addVideo"))
                    videoOn=true;
            #endif
            }
        #ifdef ADD_VIDEO
            else if (optTypeName=="video")
            {    
                type=image;
                videoOn=true;
                saveData=false;
            }
        #endif
            else
            {
                cout << "Error: invalid type" << endl;
                return false;
            }
        }
        else
            type=bottle;

        dwnsample=rf.check("downsample",Value(1)).asInt();
        rxTime=rf.check("rxTime");
        txTime=rf.check("txTime");
        string templateDirName=rf.check("dir")?rf.find("dir").asString().c_str():portName;
        if (templateDirName[0]!='/')
            templateDirName="/"+templateDirName;

        string dirName;
        if (rf.check("overwrite"))
            dirName="."+templateDirName;
        else
        {
            // look for a proper directory
            int i=0;
            char checkDirName[255];
            do
            {
                if (i>0)
                    sprintf(checkDirName,".%s_%.5d",templateDirName.c_str(),i);
                else
                    sprintf(checkDirName,".%s",templateDirName.c_str());
            
                dirName=checkDirName;
                i++;
            }
            while (!yarp::os::stat(dirName.c_str()));
        }

        createFullPath(dirName.c_str());

        q=new DumpQueue();
        t=new DumpThread(type,*q,dirName.c_str(),100,saveData,videoOn);

        if (!t->start())
        {
            delete t;
            delete q;

            return false;
        }

        reporter.setThread(t);

        if (type==bottle)
        {
            p_bottle=new DumpPort<Bottle>(*q,dwnsample,rxTime,txTime);
            p_bottle->useCallback();
            p_bottle->open(portName);
            p_bottle->setReporter(reporter);
        }
        else
        {
            p_image=new DumpPort<ImageOf<PixelBgr> >(*q,dwnsample,rxTime,txTime);
            p_image->useCallback();
            p_image->open(portName);
            p_image->setReporter(reporter);
        }

        char rpcPortName[255];
        strcpy(rpcPortName,portName);
        strcat(rpcPortName,"/rpc");

        rpcPort.open(rpcPortName);
        attach(rpcPort);

        cout << "Service yarp port: " << portName << endl;
        cout << "Data stored in   : " << dirName  << endl;

        return true;
    }

    bool close()
    {
        t->stop();

        if (type==bottle)
        {
            p_bottle->interrupt();
            p_bottle->close();
            delete p_bottle;
        }
        else
        {
            p_image->interrupt();
            p_image->close();
            delete p_image;
        }

        rpcPort.interrupt();
        rpcPort.close();

        delete t;
        delete q;

        return true;
    }

    double getPeriod()    { return 1.0;  }
    bool   updateModule() { return true; }
};


/**************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name       port: service port name (default: /dump)"                            << endl;        
        cout << "\t--dir        name: provide explicit name of storage directory"                    << endl;
        cout << "\t--overwrite      : overwrite pre-existing storage directory"                      << endl;
    #ifdef ADD_VIDEO
        cout << "\t--type       type: type of the data to be dumped [bottle(default), image, video]" << endl;
        cout << "\t--addVideo       : produce video as well (if image is selected)"                  << endl;
    #else
        cout << "\t--type       type: type of the data to be dumped [bottle(default), image]"        << endl;
    #endif
        cout << "\t--downsample    n: downsample rate (default: 1 => downsample disabled)"           << endl;
        cout << "\t--rxTime         : dump the receiver time instead of the sender time"             << endl;
        cout << "\t--txTime         : dump the sender time straightaway"                             << endl;

        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return -1;
    }

    DumpModule mod;
    return mod.runModule(rf);
}




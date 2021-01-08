#include "tracking.h"
const int MAX_NUMBER_OF_REPLIES = 50;    //!< application quits when max replies are reached
const int MAX_TRACK_FAIL_COUNT = 25;      //!< max number of attempts before target is redesignated
const int MAX_BUFFER_LEN = 256;           //!< maximum number of bytes in the buffer used to read/write to socket
bool currentImageSizeReceived=true;
bool done;
#define CAM_IDX 2 
///////////////////////////////////////////////////////////////////////////////
u16 toU16(const u8 *ptr)
{
  u16 a;
  a = ptr[0] | (ptr[1]<<8);
  return a;
}

///////////////////////////////////////////////////////////////////////////////
s16 toS16(const u8 *ptr)
{
  u16 a;
  a = ptr[0] | (ptr[1]<<8);
  return (s16)a;
}

///////////////////////////////////////////////////////////////////////////////
u32 toU32(const u8 *ptr)
{
  u32 a;
  a = ptr[0] | (ptr[1]<<8) | (ptr[2]<<16) | (ptr[3]<<24);
  return a;
}

///////////////////////////////////////////////////////////////////////////////
s32 toS32(const u8 *ptr)
{
  u32 a;
  a = ptr[0] | (ptr[1]<<8) | (ptr[2]<<16) | (ptr[3]<<24);
  return (s32)a;
}

///////////////////////////////////////////////////////////////////////////////
u64 toU64(const u8 *ptr)
{
  u32 a,b;

  a = ptr[0] | (ptr[1]<<8) | (ptr[2]<<16) | (ptr[3]<<24);
  b = ptr[4] | (ptr[5]<<8) | (ptr[6]<<16) | (ptr[7]<<24);
  return (u64)a + (((u64)b)<<32);
}





///////////////////////////////////////////////////////////////////////////////
// Functions for SL Internal Use

u32 SlFipGetPktLen(const u8 *data, s32 *extraLenBytes, bool fipEx)
{ 
  u32 len = data[0];
  *extraLenBytes = 0;
  if (fipEx) {
    if (len & MAX_SLFIPEX_BYTEMASK) {
      u32 len2 = (data[1] << 7);
      len = (len & ~MAX_SLFIPEX_BYTEMASK) + len2;
      *extraLenBytes = 1;
    }
  }
  return len;
}
static s32 simplePack(SLPacketType buffer, u8 type)
{
  u8 byteCount = SLFIPAddHeader(&buffer[0]);
  buffer[byteCount++] = 2;
  buffer[byteCount++] = type;
  buffer[byteCount++] = SLComputeFIPChecksum(&buffer[SLFIP_OFFSET_TYPE], buffer[SLFIP_OFFSET_LENGTH]-1);
  return byteCount;
}
s32 SLFIPGetOverlayMode(SLPacketType buffer)
{
  return simplePack(buffer, GetOverlayMode);
}
//---------------GET DATA FROM SLA-------------------------//
	//-----------------*GET FUNTIONS-------------------------//
	SLINLINE static u8 SLFIPAddHeader(u8 * dst)
{
  dst[SLFIP_OFFSET_HDR0] = HeaderByte1;
  dst[SLFIP_OFFSET_HDR1] = HeaderByte2;
  return 2;
}
u8 SLComputeFIPChecksum(const u8 *data, u32 len)
{
  u32 i;
  u8 crc = 1;
  for(i=0;i<len;i++){
    crc = crc8Table[ crc ^ data[i] ];
  }
  return crc;
}

	s32 SLFIPGetImageSize(SLPacketType buffer, u8 cameraIndex)
{
  u8 byteCount = SLFIPAddHeader(&buffer[0]);
  buffer[byteCount++] = 3;
  buffer[byteCount++] = GetImageSize;
  buffer[byteCount++] = cameraIndex;
  buffer[byteCount++] = SLComputeFIPChecksum(&buffer[SLFIP_OFFSET_TYPE], buffer[SLFIP_OFFSET_LENGTH] - 1);

  return byteCount;
}
s32 SLFIPGetVersionNumber(SLPacketType buffer)
{
  return SLFIPGetParameters(buffer, GetVersionNumber);
}
s32 SLFIGetCurrentOverlayObject(SLPacketType buffer)
{
	return SLFIPGetParameters(buffer, 0x3B);
}
s32 SLFIPGetParameters(SLPacketType buffer, u8 id)
{
  u8 byteCount = SLFIPAddHeader(&buffer[0]);
  buffer[byteCount++] = 3;
  buffer[byteCount++] = GetParameters;
  buffer[byteCount++] = id;
  buffer[byteCount++] = SLComputeFIPChecksum(&buffer[SLFIP_OFFSET_TYPE], buffer[SLFIP_OFFSET_LENGTH]-1);
  return byteCount;
}
///////////////////////////////////////////////////////////////////////////////
/*! A callback function to handle tracking positions received from SLA-HARDWARE
 */
s32 SLFIPUnpackTrackingPosition(SLTelemetryData *trackingPosition, const SLPacketType buffer, u64 *timeStamp, u32 *frameIdx )
{
  s32 extraLenBytes = 0;
  s32 byteCount = SLFIP_OFFSET_LENGTH;
  s32 len = (s32)SlFipGetPktLen(&buffer[byteCount], &extraLenBytes, true);

  byteCount = 4;

  trackingPosition->trackCol = toS16(&buffer[byteCount]); byteCount+=2;
  trackingPosition->trackRow = toS16(&buffer[byteCount]); byteCount+=2;

  s16 sceneCol = toS16(&buffer[byteCount]); byteCount+=2;
  s16 sceneRow = toS16(&buffer[byteCount]); byteCount+=2;

  trackingPosition->displayCol = toS16(&buffer[byteCount]); byteCount+=2;
  trackingPosition->displayRow = toS16(&buffer[byteCount]); byteCount+=2;

  trackingPosition->trackingConfidence  = buffer[byteCount++];
  trackingPosition->sceneConfidence     = buffer[byteCount++];

  trackingPosition->displayAngle7 = toU16(&buffer[byteCount]); byteCount+=2;
  // WARN: don't divide by 128 here as it will get shifted elsewhere.

  trackingPosition->idx = buffer[byteCount++];
  trackingPosition->userTrackId = buffer[byteCount++];
  
  u8 sceneColFrac8 = buffer[byteCount++];
  u8 sceneRowFrac8 = buffer[byteCount++];

  trackingPosition->sceneCol = (f32)sceneCol + (sceneColFrac8/256.0f);
  trackingPosition->sceneRow = (f32)sceneRow + (sceneRowFrac8/256.0f);

  trackingPosition->sceneAngle7 = toS16(&buffer[byteCount]); byteCount+=2;
  trackingPosition->sceneScale8 = toU16(&buffer[byteCount]); byteCount+=2;

  if (len >= 38){ // auxiliary telemetry added.
    if( timeStamp )
      *timeStamp = toU64(&buffer[byteCount]);
    byteCount += 8;
    if( frameIdx )
      *frameIdx = toU32(&buffer[byteCount]);
    byteCount += 4;
  }
  return byteCount;
}
SLStatus cbCurrentImageSize(const u8 *buf)
{
    VTImageSize img;
    u8 size = buf[2];
    if(size>=18) {
        u8 shortCount = 0;
        s16 *b = (s16*)&buf[4];
        img.capWide     = b[shortCount++];      
        img.capHigh     = b[shortCount++];      
        img.disWide     = b[shortCount++];      
        img.disHigh     = b[shortCount++];      
        img.disRectCol  = b[shortCount++];   
        img.disRectRow  = b[shortCount++];   
        img.disRectWide = b[shortCount++];  
        img.disRectHigh = b[shortCount++];
        printf("Wide: %d,High: %d, disWide:%d \n",img.capWide,img.capHigh,img.disWide);
    }
  return SLA_SUCCESS;
}
SLStatus mycbsingleTrackResult( const u8 *buffer)
{
  // Waits to receive the Current Image Size from the board before printing track information
  if (!currentImageSizeReceived)
    return SLA_FAIL;

  static int trackFailCount = 0;
  static int count = 0;


  if (count == 0) {
    printf("\n    Registration     |      Track Data   \n");
    printf(" ROW   COL   CONF    |  ROW    COL   CONF\n");
    count++;
  }

  // Quits once the count reaches the max number of replies
//  count++;
//  if (count > MAX_NUMBER_OF_REPLIES)
//  {
//    done = true;
//    return SLA_SUCCESS;
//  }

  SLTelemetryData trackPosition;
  u32 frameIdx = 0xFFFFFFFF;
  u64 timeStamp = 0;
  SLFIPUnpackTrackingPosition(&trackPosition, (SLPacketType)buffer, &timeStamp, &frameIdx);

  //
  // Print the results, or designate a new target if we loose the track
  //
  if(trackPosition.trackingConfidence == 0) { // high bit indicates fail
    //printf("% 4.2f  % 4.2f   %4d  | %4d   %4d   NO TRACK\n", trackPosition.sceneRow, trackPosition.sceneCol, trackPosition.sceneConfidence, trackPosition.trackRow, trackPosition.trackCol);
    trackFailCount++;
//    if( trackFailCount >= MAX_TRACK_FAIL_COUNT ) {
//      // Designate the target at the center of the screen
//      dataLen = SLFIPModifyTracking(outBuffer, 320, 240, 7);
//      port->Write(buffer, dataLen);
//    }
  }
  else {
    //printf("% 4.2f  % 4.2f   %4d  | %4d   %4d   %4d\n", trackPosition.sceneRow, trackPosition.sceneCol, trackPosition.sceneConfidence, trackPosition.trackRow, trackPosition.trackCol, trackPosition.trackingConfidence);
    trackFailCount = 0;
  }

  return SLA_SUCCESS;
}
////////////////////////////////////////PID////////////////////////////////////////
// this is the registration and track position of a target being tracked by VideoTrack
// translate this into a gimbal control packet and send
void GcInitState(GCPid *pid)
{
	pid->zoom = 1.0f;
	pid->pGainRow = 60;
	pid->iGainRow = 0;//4;
	pid->dGainRow = 0;//10;

	pid->pGainCol = 80;
	pid->iGainCol = 0;//3;
	pid->dGainCol = 0;//100;

	pid->trimTilt = 1.0f;
	pid->trimPan = 1.2f;
	pid->cmdX = 0;
	pid->cmdY = 0;
	}


SLStatus GimbalCommand(u16 *channels, s32 rateX, s32 centerX, s32 rateY, s32 centerY )
{
#define MIN_RATE -100
#define MAX_RATE 100
  if(rateX<MIN_RATE) rateX = MIN_RATE;
  if(rateX>MAX_RATE) rateX = MAX_RATE;
	if(rateY<MIN_RATE) rateY = MIN_RATE;
  if(rateY>MAX_RATE) rateY = MAX_RATE;
	if(rateX==-3||rateX<-3) {rateX *=10;channels[0]+=rateX;}
	else if(rateX==3||rateX>3) {rateX*=10;channels[0]+=rateX;}
	else {rateX=0;channels[0]=1023;}
	if(rateY==-3||rateY<-3) {rateY*=4;channels[1]+=rateY;}
	else if(rateY==3||rateY>3) {rateY *= 4;channels[1]+=rateY;}
	else{ rateY=0;channels[1]=1023;}

	//channels[0]+=rateX;//+centerX;
	//channels[1]+=rateY;// +centerY;
//    printf("motor PAN rateX %d, channels0 %d\n", rateX,channels[0]);
//    printf("motor PAN rateY %d,  channels1 %d\n", rateY,channels[1]);
	

  return SLA_SUCCESS;
}
SLStatus updatePidOff(GCPid *pid ,f32 errX, f32 errY)
{

  s16 wide=1920;
  s16 high=1080;
	
  f32 zoom = 1.0f;

  f32 hfov = 1.0f;
  f32 vfov = 1.0f;

  // Convert error in pixels/frame to rate in degrees/second:  *fov/dimension*zoom/framesPerSecond
  f32 rateX = (errX * hfov)/(f32)wide;
  f32 rateY = (errY * vfov)/(f32)high;

  // Update proportional
  pid->proporX  = (pid->pGainCol*rateX);
  pid->proporY  = (pid->pGainRow*rateY);

  // Update integral
  pid->integralX += (pid->iGainCol*rateX);
  pid->integralY += (pid->iGainRow*rateY);

  // Update derivative
  pid->derivX  = pid->dGainCol * (rateX - pid->lastRateX);
  pid->derivY  = pid->dGainRow * (rateY - pid->lastRateY);

  // Output command = P + I + D
  pid->cmdX = pid->proporX + pid->integralX + pid->derivX;
  pid->cmdY = pid->proporY + pid->integralY + pid->derivY;

  // Store for next step
  pid->lastRateX = rateX;
  pid->lastRateY = rateY;

  // Decay integral term a little every frame - gets rid of drifts, don't keep sums around forever
  f32 decay = .96f;  // ~29/30
  pid->integralX = pid->integralX*decay;
  pid->integralY = pid->integralY*decay;
//  if(abs(pid->integralX<=1)) pid->integralX = 0;
//  if(abs(pid->integralY<=1)) pid->integralY = 0;

//  SLTrace("e %3d %3d r % 4d % 4d p % 4d % 4d i % 4d % 4d d % 4d % 4d c % 4d % 4d\n", errX, errY,
//    rateX, rateY, pid->proporX, pid->proporY, pid->integralX, pid->integralY, pid->derivX, pid->derivY, pid->cmdX, pid->cmdY);

  return SLA_SUCCESS;
}

 SLStatus cbTrackingPosition(const u8 *buf, GCPid *pid, u16 *channels)
{

    SLTelemetryData td;
		u32 frameIdx = 0xFFFFFFFF;
		u64 timeStamp = 0;
		SLFIPUnpackTrackingPosition(&td, (SLPacketType)buf, &timeStamp, &frameIdx);
    // SLFIPUnpackTrackingPosition(&td, (SLPacketType)buf, &timeStamp, &frameIdx);
    
	//VTTrackingPosition *tp = &ctxt->state.sla.trackPos;
    //tp->trCol    = td.trackCol;
    //tp->trRow    = td.trackRow;
		//printf("% 4.2f  % 4.2f   %4d  | %4d   %4d   %4d\n", td.sceneRow, td.sceneCol, td.sceneConfidence, td.trackRow, td.trackCol, td.trackingConfidence);
    f32 trimX, trimY;
    if(pid->cmdX > 0 && pid->trimPan < 1.0f)
	{
      trimX = pid->trimPan;
    } 
	else if(pid->cmdX < 0 && pid->trimPan > 1.0f) 
	{
      trimX = 1.0f/pid->trimPan;
    } 
	else 
	{
      trimX = 1.0f;
    }

    if(pid->cmdY < 0 && pid->trimTilt < 1.0f) {
      trimY = pid->trimTilt;
    } else if(pid->cmdY > 0 && pid->trimTilt > 1.0f) {
      trimY = 1.0f/pid->trimTilt;
    } else {
      trimY = 1.0f;
    }
	s16 capWide=1920;
	s16 capHigh=1080;

    if(td.trackingConfidence>50 && td.trackCol!=-1 && td.trackRow!=-1 ) 
	{
      // Track mode gimbal control - pan/tilt towards the track
      f32 zoom =  pid->zoom;
      f32 errX = ((f32)td.trackCol - (capWide>>1))/zoom;
      f32 errY = ((f32)td.trackRow - (capHigh>>1))/zoom;

      //SLTrace("errX %f, errY %f tp->trCol %d tp->trRow %d ctxt->state.sla.img.capWide %d ctxt->state.sla.img.capHigh %d\n", errX, errY, tp->trCol, tp->trRow,
      //    ctxt->state.sla.img.capWide, ctxt->state.sla.img.capHigh);

      // TODO:  Use zoom and adjust scale factor
      updatePidOff( pid, errX, errY);

      /*SLATrace("rowpid %.f %.f %.f colpid %.f %.f %.f motor %d %d trim %.1f %.1f -> %.1f %.1f\n",
          ctxt->state.pid.pGainRow, ctxt->state.pid.iGainRow, ctxt->state.pid.dGainRow,
          ctxt->state.pid.pGainCol, ctxt->state.pid.iGainCol, ctxt->state.pid.dGainCol,
          (s32)SLROUNDZERO(ctxt->state.pid.cmdX), (s32)SLROUNDZERO(ctxt->state.pid.cmdY),
          ctxt->state.pid.trimTilt, ctxt->state.pid.trimPan, trimY, trimX);*/

      //GimbalCommand(channels,(s32)SLROUNDZERO(trimX*pid->cmdX), 115,-(s32)SLROUNDZERO(trimY*pid->cmdY),125);
     // GimbalCommand(ctxt, GimbalTilt, -(s32)SLROUNDZERO(trimY*ctxt->state.pid.cmdY), 125);
		GimbalCommand(channels,pid->cmdX, 115,pid->cmdY,125);
    }
    else 
	{
      // Stop
      //GimbalCommand(ctxt, GimbalPan, 0, 115);
      //GimbalCommand(ctxt, GimbalTilt, 0, 125);
    }
  return SLA_SUCCESS;
}
//-------------------MULTI TRACK---------------------////
s32 SLFIPUnpackTrackingPositions(u8 * cameraIndex, SLFIPTrackRes *tracks, u8 maxTracks, const SLPacketType buffer, u64 *timeStamp, u32 *frameIdx, u8 *keyIdxPress)
{
  s32 extraLenBytes = 0;
    
  for(u8 i = 0; i < 6; i++)
  {
      keyIdxPress[i]=0;
  }

  s32 byteCount = SLFIP_OFFSET_LENGTH;
  s32 len = (s32)SlFipGetPktLen(&buffer[byteCount], &extraLenBytes, true);
  byteCount += 1+extraLenBytes;

  if(buffer[byteCount++] != TrackingPositions)
    return -1;

  *cameraIndex = buffer[byteCount++];
  u8 expectedTracks = buffer[byteCount++];
  u8 mtiPayload = 0;
  if(expectedTracks&0x80) { //high bit set indicates mti payload
    mtiPayload = 1;
    expectedTracks = (expectedTracks&(~0x80));
  }

  if(expectedTracks>maxTracks) return -1;

  //SLFIPTrackRes *tracks = (SLFIPTrackRes *)trks;

  s32 numTracks = 0;
  for(u8 idx=0; (idx < expectedTracks) && (byteCount < len); idx++, numTracks++) {
    u8 idexTemp = buffer[byteCount++] + 1;
      
    //tracks[idx].idx = buffer[byteCount++];
    tracks[idexTemp].idx = idexTemp;
     
    keyIdxPress[idexTemp] = idexTemp; //*-[1,3,5]
    
    tracks[idexTemp].col = toS16(&buffer[byteCount]); byteCount += 2;
    tracks[idexTemp].row = toS16(&buffer[byteCount]); byteCount += 2;
    tracks[idexTemp].wide = toU16(&buffer[byteCount]); byteCount += 2;
    tracks[idexTemp].high = toU16(&buffer[byteCount]); byteCount += 2;

    tracks[idexTemp].velCol8 = toS16(&buffer[byteCount]); byteCount += 2;
    tracks[idexTemp].velRow8 = toS16(&buffer[byteCount]); byteCount += 2;
    
    tracks[idexTemp].confidence = buffer[byteCount++];
    
    tracks[idexTemp].flags    = buffer[byteCount]&0x83;
    tracks[idexTemp].mti      = mtiPayload;

    tracks[idexTemp].resultState = (buffer[byteCount]>>2)&0x3;
    tracks[idexTemp].frame       = 7; // >=7 here makes it draw as a regular track box

    tracks[idexTemp].userTrackId = 0; // see SLFIPUnpackTrackingPositionsExtended for supported userTrackID.
    byteCount++;
//	printf("\nTrack Index: %4d Row: %4.2f Col: %4.2f Confidence: %4d", tracks[idx].idx, tracks[idx].row, tracks[idx].col, tracks[idx].confidence);

  }
//   printf("\n");
//  for(u8 i = 0; i < 6; i++)
//  {
//      printf("%d-", keyIdxPress[i]);
//  }
//  printf("\n");
  if( len-byteCount >= 10 ){
    if( timeStamp )
      *timeStamp = toU64(&buffer[byteCount]);
    byteCount += 8;
    if( frameIdx )
      *frameIdx = toU32(&buffer[byteCount]);
    byteCount += 4;
  }
  return numTracks;
}

SLStatus mycbTrackPositions(SLFIPTrackRes *tracks, const u8 *buffer, s32 *numTracks,u8 *keyIdxPress)
{
	//printf("ok");
  u32 frameIdx = 0xFFFFFFFF;
  u64 timeStamp = 0;
  u8 maxTracks = SLFIP_MAX_TOTAL_TRACK_RES;
  u8 cameraIndex = CAM_IDX;
  *numTracks = SLFIPUnpackTrackingPositions(&cameraIndex, tracks, maxTracks, (SLPacketType) buffer, &timeStamp, &frameIdx, keyIdxPress);
  //s32 numTracks = SLFIPUnpackTrackingPositionsExtended(&cameraIndex, (void*)tracks, maxTracks, (SLPacketType)buffer, &timeStamp, &frameIdx);
  // Uncomment this to check if you are receiving tracking positions information
  // Prints the first track if there is more than one track
 /// if (*numTracks > 0){
//      printf("\n numtrack: %d", *numTracks);
//  }

//  for(u8 idx = 0; idx < *numTracks; idx++) {
//    printf("\nTrack Index: %4d Row: %4.2f Col: %4.2f Confidence: %4d", tracks[idx].idx, tracks[idx].row, tracks[idx].col, tracks[idx].confidence);
//  }
   //printf("\n\n\nTrack Index: %4d Row: %4.2f Col: %4.2f Confidence: %4d", tracks->idx, tracks->row, tracks->col, tracks->confidence);

  return SLA_SUCCESS;
}
SLStatus TrackingControl(SLFIPTrackRes td, GCPid *pid, u16 *channels)
{
    //SLTelemetryData td;
    u32 frameIdx = 0xFFFFFFFF;
    u64 timeStamp = 0;
    f32 trimX, trimY;
	//SLFIPUnpackTrackingPosition(&td, (SLPacketType)buf, &timeStamp, &frameIdx);
    if(pid->cmdX > 0 && pid->trimPan < 1.0f)
	{
      trimX = pid->trimPan;
    } 
	else if(pid->cmdX < 0 && pid->trimPan > 1.0f) 
	{
      trimX = 1.0f/pid->trimPan;
    } 
	else 
	{
      trimX = 1.0f;
    }

    if(pid->cmdY < 0 && pid->trimTilt < 1.0f) {
      trimY = pid->trimTilt;
    } else if(pid->cmdY > 0 && pid->trimTilt > 1.0f) {
      trimY = 1.0f/pid->trimTilt;
    } else {
      trimY = 1.0f;
    }
	s16 capWide=1920;
	s16 capHigh=1080;
    if(td.confidence>50 && td.col!=-1 && td.row!=-1 ) 
	{
      // Track mode gimbal control - pan/tilt towards the track
      f32 zoom =  pid->zoom;
      f32 errX = ((f32)td.col - (capWide>>1))/zoom;
      f32 errY = ((f32)td.row - (capHigh>>1))/zoom;
      updatePidOff( pid, errX, errY);
		GimbalCommand(channels,pid->cmdX, 115,pid->cmdY,125);
    }
	//printf("\nTrack Index: %4d Row: %4.2f Col: %4.2f Confidence: %4d", td.idx, td.row, td.col, td.confidence);

  return SLA_SUCCESS;
}
#ifndef __TRACKING_H
#define __TRACKING_H
#include <stdio.h>
#include <stdbool.h>
#define u8  unsigned char    //!< 8 bit unsigned integer
#define s16 short           //!< 16 bit signed integer
#define u16 unsigned short  //!< 16 bit unsigned integer
#define s32 int             //!< 32 bit signed integer
#define u32 unsigned int //!< 32 bit unsigned integer
#define s64 long long
#define u64 unsigned long long
#define f32 float
#define MAX_SLFIPEX_BYTEMASK  (0x80) //!< Maximum length for a single byte length field for FIPEX.
#define SLINLINE __inline //!< Indicates to compiler that function should be inlines if possible
typedef u8 * SLPacketType;
/* write SBUS packets */
/*! Byte offsets 
 */
#define SLFIP_OFFSET_HDR0   0
#define SLFIP_OFFSET_HDR1   1
#define SLFIP_OFFSET_LENGTH 2
#define SLFIP_OFFSET_TYPE   3
static const u8 crc8Table[ ] =
{
    0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
  157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
   35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
  190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
   70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
  219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
  101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
  248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
  140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
   17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
  175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
   50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
  202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
   87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
  233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
  116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
} ;
typedef enum {
  SLA_ERROR     = -1,
  SLA_SUCCESS   = 0,
  SLA_FAIL      = 1,
  SLA_TIMEOUT   = 2,
  SLA_LICENSE   = 3,
  SLA_TERMINATE = 4,
  SLA_NOP       = 5,
  SLA_NOT_SENT  = 6,
  SLA_ALLOC     = 7,
  SLA_ERR_ARG   = 8,
  SLA_ERR_RANGE = 9,
  SLA_ERR_FMT   = 10,
  SLA_VERSION   = 11,
  SLA_NOT_FOUND = 12
} SLStatus;

typedef struct {
  s16 trackCol;
  s16 trackRow;
  f32 sceneCol;
  f32 sceneRow;
  s16 displayCol;
  s16 displayRow;
  u8 trackingConfidence;
  u8 sceneConfidence;
  u16 displayAngle7;
  u8 idx;
  u8 userTrackId;
  s16 sceneAngle7;
  u16 sceneScale8;
} SLTelemetryData;

typedef struct {
  s16 capWide;
  s16 capHigh;
  s16 disWide;
  s16 disHigh;
  s16 disRectCol;   
  s16 disRectRow;   
  s16 disRectWide;  
  s16 disRectHigh;
} VTImageSize;
u32 SlFipGetPktLen(const u8 *data, s32 *extraLenBytes, bool fipEx);
static s32 simplePack(SLPacketType buffer, u8 type);
s32 SLFIPGetOverlayMode(SLPacketType buffer);
s32 SLFIPUnpackTrackingPosition(SLTelemetryData *trackingPosition, const SLPacketType buffer, u64 *timeStamp, u32 *frameIdx );
SLStatus mycbsingleTrackResult( const u8 *buffer);
//s32 SLFIPGetImageSize(SLPacketType buffer);
s32 SLFIPGetImageSize(SLPacketType buffer, u8 cameraIndex);
SLINLINE static u8 SLFIPAddHeader(u8 * dst);
u8 SLComputeFIPChecksum(const u8 *data, u32 len);
//SLStatus mycbCurrentImageSizeResult(void *context, const u8 *buffer);
//static SLStatus getAllFromSla();
s32 SLFIPGetVersionNumber(SLPacketType buffer);
s32 SLFIPGetParameters(SLPacketType buffer, u8 id);
s32 SLFIGetCurrentOverlayObject(SLPacketType buffer);
SLStatus cbCurrentImageSize(const u8 *buf);
typedef enum SLFIP {
  // Header signature byte 1
  HeaderByte1 = 0x51,
  HeaderByte2 = 0xAC,
	_sbusHeader = 0x0F,
	_sbusFooter = 0x00,
  // Commands
  FirstFipNumber                                    = 0x00,
  GetVersionNumber                                  = 0x00,
  ResetAllParameters                                = 0x01,
  GetConfiguration                                  = 0x01, // Used as ID in Get Parameters Function(0x28) and its response.
  SetStabilizationParameters                        = 0x02,
  GetStabilizationParameters                        = 0x03,
  ResetStabilizationMode                            = 0x04,
  ResetStabilizationParameters                      = 0x04,
  ModifyTracking                                    = 0x05,
  SetOverlayMode                                    = 0x06,
  GetOverlayMode                                    = 0x07,
  StartTracking                                     = 0x08,
  StopTracking                                      = 0x09,
  NudgeTrackingCoordinate                           = 0x0A,
  SetCoordinateReportingMode                        = 0x0B,
  CoordinateReportingMode                           = 0x0B, // Alias
  SetTrackingParameters                             = 0x0C,
  GetTrackingParameters                             = 0x0D,
  SetRegistrationParameters                         = 0x0E,
  GetRegistrationParameters                         = 0x0F,
  SetVideoParameters                                = 0x10,
  SetAVideoParameters                               = SetVideoParameters, // Deprecated, use SetVideoParameters
  GetVideoParameters                                = 0x11,
  GetAVideoParameters                               = GetVideoParameters, // Deprecated, use GetVideoParameters
  SetStabilizationBias                              = 0x12,
  SetMetadataValues                                 = 0x13,
  MetadataStaticValues                              = 0x14,
  SetMetadataFrameValues                            = 0x15,
  SetDisplayParameters                              = 0x16,
  SetDisplayRotation                                = SetDisplayParameters,  // Deprecated, use SetDisplayParameters
  ModifyTrackIndex                                  = 0x17,
  SetADCParameters                                  = 0x18,
  GetADCParameters                                  = 0x19,
  SetEthernetVideoParameters                        = 0x1A,
  GetEthernetVideoParameters                        = 0x1B,
  SetNetworkParameters                              = 0x1C,
  GetNetworkParameters                              = 0x1D,
  SetSDRecordingParameters                          = 0x1E,
  SetVideoMode                                      = 0x1F,
  GetVideoMode                                      = 0x20,
  SetVideoEnhancementParameters                     = 0x21,
  GetVideoEnhancementParameters                     = 0x22,
  SetH264Parameters                                 = 0x23,
  GetH264Parameters                                 = 0x24,
  SaveParameters                                    = 0x25,
  Obsoleted_SetRGB565Conversion                     = 0x26, // 2000 functionality only
  Obsoleted_CurrentPrimaryTrackIndex                = 0x27, // Deprecated - use telemetry to determine primary
  GetParameters                                     = 0x28,
  SetEthernetDisplayParameters                      = 0x29,
  SetDisplayAdjustments                             = 0x2A,
  Obsoleted_SetStitchParameters                     = 0x2B, // 2000 only
  Obsoleted_GetStitchParameters                     = 0x2C, // 2000 only
  SetMovingTargetDetectionParameters                = 0x2D,
  SetDetectionParameters                            = 0x2D, // Alias
  GetMovingTargetDetectionParameters                = 0x2E,
  GetDetectionParameters                            = 0x2E, // Alias
  SendBlendParams                                   = 0x2F,
  SetBlendParameters                                = 0x2F, // Alias
  GetBlendParams                                    = 0x30,
  GetBlendParameters                                = 0x30, // Alias
  GetImageSize                                      = 0x31,
  DesignateSelectedTrackPrimary                     = 0x32,
  ShiftSelectedTrack                                = 0x33,
  Obsoleted_GetPrimaryTrackIndex                    = 0x34, // Deprecated, use telemetry to determine primary  
  NucParameters                                     = 0x35,
  ReadWriteNuc                                      = 0x36,
  SetAcqParams                                      = 0x37,
  SetAcquisitionParameters                          = 0x37, // Alias 
  GetAcqParams                                      = 0x38,
  GetAcquisitionParameters                          = 0x38, // Alias
  GetEthernetDisplayParameters                      = 0x39,
  GetDisplayParameters                              = 0x3A,
  GetDisplayRotation                                = GetDisplayParameters, // Deprecated, use GetDisplayParameters
  DrawObject                                        = 0x3B,
  StopSelectedTrack                                 = 0x3C,
  CommandPassThrough                                = 0x3D,
  SetPortConfiguration                              = 0x3E,
  ConfigureCommunicationsPort                       = 0x3E,
  SetSerialPassthrough                              = 0x3E, // Alias
  GetPortConfiguration                              = 0x3F,
  GetPortConfig                                     = 0x3F,
  GetSerialPassthrough                              = 0x3F, // Alias
  VersionNumber                                     = 0x40,
  CurrentStabilizationMode                          = 0x41,
  CurrentStabilizationParameters                    = 0x41, // Alias
  CurrentOverlayMode                                = 0x42,
  TrackingPosition                                  = 0x43,
  CurrentTrackingParameters                         = 0x44,
  CurrentRegistrationParameters                     = 0x45,
  CurrentVideoParameters                            = 0x46,
  CurrentAVideoParameters                           = CurrentVideoParameters, // Deprecated, use CurrentVideoParameters
  CurrentADCParameters                              = 0x47,
  CurrentEthernetVideoParameters                    = 0x48,
  CurrentNetworkParameters                          = 0x49,
  CurrentVideoEnhancementParameters                 = 0x4A,
  CurrentVideoModeParameters                        = 0x4B,
  Obsoleted_CurrentStitchParameters                 = 0x4C, // 2000 only
  CurrentBlendParameters                            = 0x4D,
  CurrentImageSize                                  = 0x4E,
  CurrentAcqParams                                  = 0x4F,
  CurrentAcquisitionParameters                      = 0x4F, // Alias
  GetHardwareID                                     = 0x50,
  TrackingPositions                                 = 0x51,
  CurrentEthernetDisplayParameters                  = 0x52,
  CurrentPortConfiguration                          = 0x53,
  CurrentMovingTargetDetectionParameters            = 0x54,
  CurrentDetectionParameters                        = 0x54,
  FocusStats                                        = 0x55,
  CurrentH264Parameters                             = 0x56,
  CurrentDisplayParameters                          = 0x57,
  CurrentDisplayRotation                            = CurrentDisplayParameters, // Deprecated, use CurrentDisplayParameters
  CurrentSDCardRecordingStatus                      = 0x58,
  CurrentSDCardDirectoryInfo                        = 0x59,
  SendTraceStr                                      = 0x5A,
  SetCommandCamera                                  = 0x5B, // was SendMTIDebugInfo
  CommandCamera                                     = 0x5B, // Alias
  SetDisplayAngle                                   = 0x5C,
  DisplayAngle                                      = 0x5C, // Alias
  CurrentSnapShot                                   = 0x5D,
  SetSnapShot                                       = 0x5E,
  GetSnapShot                                       = 0x5F,
  DoSnapShot                                        = 0x60,
  SetKlvData                                        = 0x61,
  SetMetadataRate                                   = 0x62,
  SetSystemType                                     = 0x63,  // was SetDualBoardMode
  SetTelemetryDestination                           = 0x64,  // was GetDualBoardMode
  CurrentSystemType                                 = 0x65,  // was CurrentDualBoardMode
  NetworkList                                       = 0x66,
  GetNetworkList                                    = 0x66,  // Alias
  CurrentNetworkList                                = 0x67,
  CurrentOverlayObjectsIds                          = 0x68, // returns the IDs of displayed objects.
  Obsoleted_SetParameterBlock                       = 0x69,
  Obsoleted_ParameterBlock                          = 0x6A,
  CurrentOverlayObjectParams                        = 0x6B, // all the data associated with a displayed object
  CurrentOverlayObjectParameters                    = 0x6B, // Alias
  SetLensMode                                       = 0x6C,
  CurrentLensStatus                                 = 0x6D,
  SetLensParams                                     = 0x6E,
  SetLensParameters                                 = 0x6E, // Alias
  CurrentLensParams                                 = 0x6F,
  CurrentLensParameters                             = 0x6F, // Alias
  SetDigCamParams                                   = 0x70,
  SetDigitalCameraParameters                        = 0x70, // Alias
  CurrentDigCamParams                               = 0x71,
  CurrentDigitalCameraParameters                    = 0x71, // Alias
  SetUserPalette                                    = 0x72,
  CurrentUserPalette                                = 0x73,
  GetUserPalette                                    = 0x73, // Alias
  SetMultipleAlignment                              = 0x74,
  CurrentMultipleAlignment                          = 0x75,
  GetMultipleAlignment                              = 0x75, // Alias
  SetAdvancedMoTDetParameters                       = 0x76,
  SetAdvancedDetectionParameters                    = 0x76, // Alias
  CurrentAdvancedMoTDetParameters                   = 0x77,
  CurrentAdvancedDetectionParameters                = 0x77, // Alias
  GetAdvancedDetectionParameters                    = 0x77, // Alias
  TrackingBoxPixelStats                             = 0x78,
  DirectoryStatisticsReply                          = 0x79,
  CurrentStabilizationBias                          = 0x7A,
  SetAdvancedCaptureParams                          = 0x7B,
  AdvancedCaptureParameters                         = 0x7B, // Alias
  SetMotDetRegionOfInterestParams                   = 0x7C,
  SetDetectionRegionOfInterestParameters            = 0x7C, // Alias
  CurrentMotDetRegionOfInterestParams               = 0x7D,
  CurrentDetectionRegionOfInterestParameters        = 0x7D, // Alias
  GetDetectionRegionOfInterestParameters            = 0x7D, // Alias
  Obsoleted_AnalyzeRenderSync                       = 0x7E, // Deprecated
  UserWarningLevel                                  = 0x7F,
  SetUserWarningLevel                               = 0x7F, // Alias
  SystemStatusMode                                  = 0x80,
  LandingAid                                        = 0x81,
  CameraSwitch                                      = 0x82,
  LandingPosition                                   = 0x83,
  SetVMTI                                           = 0x84,  // Was MetadataOverlays
  Obsoleted_SetGeneric                              = 0x85, // Deprecated - see SetSystemValue.
  UserWarningMessage                                = 0x86,
  SystemStatusMessage                               = 0x87,
  DetailedTimingMessage                             = 0x88,
  AppendedMetadata                                  = 0x89,
  FrameIndex                                        = 0x8A,
  CurrentMetadataValues                             = 0x8B,
  CurrentMetadataFrameValues                        = 0x8C,
  CurrentMetadataRate                               = 0x8D,
  CurrentConfiguration                              = 0x8E,
  ExternalProgram                                   = 0x8F,
  StreamingControl                                  = 0x90,
  DigitalVideoParserParameters                      = 0x91,
  SetSystemValue                                    = 0x92,
  CurrentSystemValue                                = 0x93,
  I2CCommand                                        = 0x94,
  FourAlignPoints                                   = 0x95,
  TagData                                           = 0x96,
  TagDataRate                                       = 0x97, 
  TagSourceSelector                                 = 0x98,
  DecoderParameters                                 = 0x99,
  Obsoleted_CurrentDecoderParameters                = 0x9A,  // Obsoleted
  LogoParameters                                    = 0x9B,
  DrawOverlay                                       = 0x9C,
  TrackTrails                                       = 0x9D,
  RegistrationParameters                            = 0x9E,
  StabilizationBias                                 = 0x9F,
  TrackingPositionsExtended                         = 0xA0,
  DeadPixelStats                                    = 0xA1,
  InternalCommand                                   = 0xA2, // For SLALib commands.
  InternalResponse                                  = 0xA3, // For SLALib responses.
  VideoDisplay                                      = 0xA4,
  MultiDisplay                                      = 0xA5,
  Usb3VisionFeature                                 = 0xA6,
  CustomClassifier                                  = 0xA7,
  DeadPixel                                         = 0xA8,
  ClassifierParameters                              = 0xA9,
  SendScript                                        = 0xAA, // For mostly SLALib(?)
  DoDetectSnapShot                                  = 0xAB,
  AncillaryTextMetadata                             = 0xAC,
  LastFipNumber,                                            // NOTE: No more individual "getter" functions are needed @see GetParameters
                                                            // However, different codes for "Set*" and "Current*" messages ARE needed!
  FIP_EXTENDED_TYPE                                 = 0xFF, // Will need to add extended type handling beyond this point
  NumberOfFipMessages = LastFipNumber-FirstFipNumber
} SLA_FIP_RESPONSES;
///////////-------PID-----------////////////////////////////
#define SLROUNDZERO(x) ((x)>0?((x)+0.5f):((x)-0.5f))
typedef struct {
  f32 proporX, proporY;
  f32 integralX, integralY;
  f32 derivX, derivY;
  f32 cmdX, cmdY;
  f32 lastRateX, lastRateY;
  f32 pGainRow;
  f32 dGainRow;
  f32 iGainRow;
  f32 pGainCol;
  f32 dGainCol;
  f32 iGainCol;
  f32 trimTilt;
  f32 trimPan;
  f32 zoom;
} GCPid;
typedef enum {
  GimbalPan = 1,
  GimbalTilt = 2,
  GimbalMove = 3
} GimbalMotor;
void GcInitState(GCPid *pid);
void SBUS_write(u16* channels);
SLStatus GimbalCommand(u16 *channels, s32 rateX, s32 centerX, s32 rateY, s32 centerY );
SLStatus updatePidOff(GCPid *pid ,f32 errX, f32 errY);
SLStatus cbTrackingPosition(const u8 *buf, GCPid *pid, u16 *channels);
//-------------------------MULTITRACKING--------------------------//
#define SLFIP_MAX_DRAW_OBJECTS 128
#define SLFIP_MTI_MAX_TRACKED_OBJECTS 100
#define SLFIP_MAX_TRACK_RES 10
#define SLFIP_MAX_TOTAL_TRACK_RES (SLFIP_MAX_TRACK_RES + SLFIP_MTI_MAX_TRACKED_OBJECTS)
typedef struct {
  f32 row;              //!< Row at which model was found
  f32 col;              //!< Column at which model was found
  s16 high;             //!< Model height (may differ from startArgs when searching in scale)
  s16 wide;             //!< Model width (may differ from startArgs when searching in scale)
  s16 angle7;           //!< Angle at which the model was found (-180 to +180 degrees)*128
  s16 scale8;           //!< Scale at which the model was found * 256.  (1.0*256 inidicates no change)
  s16 velRow8;          //!< Track momentum row - velocity or position depending on track mode
  s16 velCol8;          //!< Track momentum row - velocity or position depending on track mode
  s16 frame;            //!< Nth frame processed
  u8 confidence;        //!< Confidence level (0...100) of model found
  u8 resultState;       //!< indicates current state of track result
  u8 flags;             //!< Primary/selected flags - 0x01 primary bit, 0x02 selected bit
  u8 idx;               // Index of original track
  u8 classifierConf;
  u8 mti;               // Mti payload (See SL_MTI_RES_TYPES)
  u8 classifierId;
  u8 userTrackId;       // for multiple target management by user.
  u8 trackColFrac8;     // 1/256ths of the track column.
  u8 trackRowFrac8;     // 1/256ths of the track row.
} SLFIPTrackRes;
s32 SLFIPUnpackTrackingPositions(u8 * cameraIndex, SLFIPTrackRes *trks, u8 maxTracks, const SLPacketType buffer, u64 *timeStamp, u32 *frameIdx, u8 *keyIdxPress);
SLStatus mycbTrackPositions(SLFIPTrackRes *tracks, const u8 *buffer,s32 *numTracks, u8 *keyIdxPress);
SLStatus TrackingControl(SLFIPTrackRes td, GCPid *pid, u16 *channels);
#endif

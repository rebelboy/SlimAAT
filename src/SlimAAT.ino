#include <Servo.h>
#include <mavlink.h>                    /* Mavlink interface */

#define azPWM_Pin  9               /* azimuth servo */
#define elPWM_Pin 11              /* elevation servo */


uint8_t minDist   = 4;            /* 距家的最小距离 */
bool  new_GPS_data  = false;        /*有新的gps数据 */
uint8_t fix_type  = 0;            /* gps定位类型 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK.GPS修正类型 */
bool  homeInitialised = false;        /*有没有设置家 */


/*  variables for servos */
int16_t azPWM   = 0;
int16_t elPWM   = 0;
int16_t LastGoodpntAz = 90;
int16_t LastGoodEl  = 0;

/* Create Servo objects */
Servo azServo;        /* Azimuth 水平舵机 */
Servo elServo;        /* Elevation //俯仰舵机 */

struct Location {
  float lat;    /* long */
  float lon;
  float alt;
  float hdg;
};

struct Location hom = {
  0, 0, 0, 0
};      /* home location */

struct Location cur = {
  0, 0, 0, 0
};      /* current location */


struct Vector {
  float az;
  float el;
  int32_t dist;
};
/* Vector for home-to-current location */
struct Vector hc_vector = {
  90, 0, 0
};


void setup()
{
  azServo.attach( azPWM_Pin );
  elServo.attach( elPWM_Pin );
  PositionServos( 90, 0, 90 );            /* Intialise servos to az=90, el=0, hom.hdg = 90; */

  uint8_t cpsGood = Initialise_Compass(); /* Check if we have a compass on the Tracker  使用罗盘 */
  if ( !(cpsGood) )                       /* 检测罗盘是否存在 */
  {
    Serial.println( "No compass found!" );
  }


 TestServos();

  Serial.begin( 57600 );
}


void TestServos()
{
  PositionServos( 90, 0, 90 );
  for ( int i = 1; i <= 360; i++ )
  {
    delay( 60 );
    PositionServos( i, 30, 90 );
  }
  for ( int i = 1; i <= 170; i++ )
  {
    delay( 60 );
    PositionServos( 90, i, 90 );
  }
  PositionServos( 90, 0, 90 );
}


void loop()
{


  Mavlink_Receive();

  /* float fHeading = GetMagHeading(); */

  if ( homeInitialised )
  {
    Serial.println("gps信息 ");
    Serial.print("家 lat: ");//in degrees * 1E7，纬度，单位为度数*10的7次方
    Serial.print(hom.lat);
    Serial.print("lon: ");//in degrees * 1E7，经度，单位为度数*10的7次方。
    Serial.print(hom.lon);
    Serial.print(" 当前点  lat: ");//in degrees * 1E7，纬度，单位为度数*10的7次方
    Serial.print(cur.lat);
    Serial.print("lon: ");//in degrees * 1E7，经度，单位为度数*10的7次方。
    Serial.println(cur.lon);

    
    GetAzEl( hom, cur );                                            /* ***************计算坐标和夹角************* */

    if ( hc_vector.dist >= minDist )
    {
      Serial.print("移动 ");
      Serial.print("水平 =>");
      Serial.println(hc_vector.az);
      Serial.print("俯仰 =>");
      Serial.println(hc_vector.el);
      Serial.print("指南针 =>");
      Serial.println(hom.hdg);
      PositionServos( hc_vector.az, hc_vector.el, hom.hdg);  /* Relative to home heading *************移动舵机位置**************** */
      new_GPS_data = false;
    }
  } else{
    /* 设置家 */
    FinalStoreHome();
  }
}


void FinalStoreHome()
{

  if(new_GPS_data==false)
    return;
  
  /* 自己有罗盘的方式  hdg就是罗盘的角度 */
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;
  hom.hdg = GetMagHeading(); /* From own compass */

  homeInitialised = true;

  Serial.print("已设置家 =>");
  /*
   * DisplayHome();
   * break;
   */
}


void Mavlink_Receive()
{
  mavlink_message_t msg;
  mavlink_status_t  status;


  while ( Serial.available() > 0 )
  {
    uint8_t c = Serial.read();

    /* Try to get a new message */
    if ( mavlink_parse_char( MAVLINK_COMM_0, c, &msg, &status ) )
    {
      /* Handle message */

      switch ( msg.msgid )
      {
      case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode( &msg, &gps );

        cur.lat   = (float) gps.lat / 1E7;
        cur.lon   = (float) gps.lon / 1E7;
        cur.alt   = gps.alt / 1E3;
        fix_type  = gps.fix_type; /* 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK.GPS修正类型 */

        
        
//                       Serial.print("MAVLINK_GPS_RAW_INT =>");
//                     Serial.print("lat: ");//in degrees * 1E7，纬度，单位为度数*10的7次方
//                      Serial.print(gps.lat);
//                      Serial.print("lon: ");//in degrees * 1E7，经度，单位为度数*10的7次方。
//                       Serial.print(gps.lon);
//                      Serial.print("alt: ");//高度，单位为km，向上为正
//                      Serial.print(gps.alt);
//                      Serial.print("fix_type: ");//0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK.GPS修正类型
//                      Serial.println(gps.fix_type);

      }
      break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      {
        if ( fix_type < 3 )
          break;

        mavlink_global_position_int_t gps;
        mavlink_msg_global_position_int_decode( &msg, &gps );

        new_GPS_data = true;

        cur.lat = (float) gps.lat / 1E7;
        cur.lon = (float) gps.lon / 1E7;
        cur.alt = gps.alt / 1E3;
        cur.hdg = gps.hdg / 100;



//                      Serial.print("MAVLINK_MSG_ID_GLOBAL_POSITION_INT =>");
//                      Serial.print("lat: ");//in degrees * 1E7，纬度，单位为度数*10的7次方
//                       Serial.print(cur.lat);
//                      Serial.print("lon: ");//in degrees * 1E7，经度，单位为度数*10的7次方。
//                       Serial.println(cur.lon);
         
        
      }
      break;
      default:
        /* Do nothing */
        break;
      }
    }

    /* And get the next one */
  }
}

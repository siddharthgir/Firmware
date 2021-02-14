/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PreFlightCheck.cpp
 */

#include "PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_gps_position.h>
#include<stdlib.h>
#include<time.h>
#include<stdio.h>
#include<string.h>


struct Date_time{
       int Hours;
       int Minutes;
       int Seconds;
       int date;
       int Month;
       int Year;
};
struct GEO_DATE_TIME_XML{// for storing xml data
    Date_time start;
    Date_time end;
};

struct ParsedData{
   char* start_time;
   char* end_time;
   float long_lat_coords[20][10];
   int num_coords;
   char certificate[1100];
};

GEO_DATE_TIME_XML parseTOgdt(ParsedData D){
	GEO_DATE_TIME_XML Rv;
	char *p,*q;
	p=D.start_time;
	q=D.end_time;
        char df[4];
        memcpy(df,p,4);
        Rv.start.Year=atoi(df);
        char mf[2];
        memcpy(mf,(p+5),2);
	Rv.start.Month=atoi(mf)/10000;
	memcpy(mf,(p+8),2);
	Rv.start.date=atoi(mf)/10000;



	memcpy(df,q,4);
	Rv.end.Year=atoi(df);
	memcpy(mf,(q+5),2);
	Rv.end.Month=atoi(mf)/10000;
	memcpy(mf,(q+8),2);
	Rv.end.date=atoi(mf)/10000;





	memcpy(mf,(p+11),2);
	Rv.start.Hours=atoi(mf)/10000;
	memcpy(mf,(p+14),2);
	Rv.start.Minutes=atoi(mf)/10000;
	memcpy(mf,(p+17),2);
	Rv.start.Seconds=atoi(mf)/10000;


	memcpy(mf,(q+11),2);
	Rv.end.Hours=atoi(mf)/10000;
	memcpy(mf,(q+14),2);
	Rv.end.Minutes=atoi(mf)/10000;
	memcpy(mf,(q+17),2);
	Rv.end.Seconds=atoi(mf)/10000;

        return Rv;
}
char* strip(char *str) {
    size_t len = strlen(str);
    memmove(str, str+1, len-2);
    str[len-2] = 0;
    return str;
}

int length(char a[1000]){
    int count=0;
while(a[ count ] != '\0'){
    count=count+1;
}
return count;

}/* Driver code */

int isSubstring(char s1[50], char s2[100])///s1 is the sub string ; s2 is the larger string
{
    int M = length(s1);
    int N = length(s2);

    for (int i = 0; i <= N - M; i++) {
        int j;

        for (j = 0; j < M; j++)
            if (s2[i + j] != s1[j])
                break;

        if (j == M)
            return i;
    }

    return -1;
}

bool In_Time(Date_time current, GEO_DATE_TIME_XML Xml){

	// full check
    printf("hello hello");
	if(Xml.start.Year<=current.Year && current.Year<=Xml.end.Year){
        printf("year is ok");
		if(Xml.start.Month<=current.Month && current.Month<=Xml.end.Month){
            printf("month is ok");
                       if(Xml.start.date<=current.date && current.date<=Xml.end.date){
                           printf("date is ok");

		       }
		       else{ printf("%d",current.Year);
			       return 0;
		       }
		}else{
			return 0;
		}
	}
	else{
        	return 0;
	}
	//time check
	if(Xml.start.Hours<=current.Hours||current.Hours<=Xml.end.Hours){
        printf("\nhours is ok");
        if(Xml.start.Hours==current.Hours||current.Hours==Xml.end.Hours){
            printf("both the hours are equal");
            if(Xml.start.Hours==current.Hours){
                printf("\nstart hours are equal");
                if(Xml.start.Minutes<=current.Minutes){
                    printf(" minutes are ok");
                    if(Xml.start.Minutes==current.Minutes){
                        printf("miutes are equal");
                    if(Xml.start.Seconds<=current.Seconds){
                        return 1;
                    }else{return 0;}
                }else{return 1;}
                }else{return 0;}
            }
            else{
                 if(Xml.end.Minutes>=current.Minutes){
                    if(Xml.end.Minutes==current.Minutes){
                    if(Xml.end.Seconds>current.Seconds){
                        return 1;
                    }else{return 0;}
                }else{return 1;}
                }else{return 0;}

            }
        }
}else{return 0;}
return 0;
}

ParsedData parse_artifact()
{
   FILE *fp;

   ParsedData result{};

   fp = fopen("./log/permission_artifact_breach.xml", "r"); // read mode

    char xml_filename[] = "./log/permission_artifact_breach.xml";
    char txt_filename[] = "./log/test.txt";
    char copy_command[256] = "cp ";
    strcat(copy_command,xml_filename);
    strcat(copy_command," ");
    strcat(copy_command,txt_filename);
    printf("\n");
    printf("%s",copy_command);
    int errorcode = system(copy_command);
    printf(" error code %d ",errorcode);

   char certi_tagi[30]="<X509Certificate>";
   char certi_tage[30]="</X509Certificate>";

   char buf[1000], start_time[200], end_time[200],long_lat_coords[20][20];
   rewind(fp);
   int c_flag_i=0,c_flag_e=0,line;
   line=0;
   int index[2][2]={{0,0},{0,0}};
   int coord_ind = 0;
   int let =0;
   (void)let;
   int aux=0;
   while(fscanf(fp, "%s", buf) != EOF )
		{  line=line+1;
      if(isSubstring(certi_tagi,buf)!=-1 || isSubstring(certi_tage,buf)!=-1 ){

       if (isSubstring(certi_tagi,buf)!=-1){       //for "<X509Certificate>"
           index[0][0]=line;
           index[0][1]=isSubstring(certi_tagi,buf);
           c_flag_i=1;
           let =length(buf);
           aux=1;
       }
       if(isSubstring(certi_tage,buf)!=-1){                          //for "</X509Certificate>"
           index[1][0]=line;
           index[1][1]=isSubstring(certi_tage,buf);
           c_flag_e=1;

       }
     //   printf("length %d\n",length(buf));

      }
         if(c_flag_i==1 && c_flag_e==0){
         if (aux==1){
         for(int u=index[0][1]+17;u<length(buf);u++){
        //   printf("%c",buf[u]);
	     result.certificate[u]=buf[u];
         }
         aux=0;
         }
         else{
           // printf("%s\n",buf);
         }
         }
    //     printf("%s\n",buf);
         int succ = 0;
         succ = sscanf(buf,"flightEndTime=%s",buf);
			if (succ != 0)strcpy(end_time,strip(buf));
         succ = sscanf(buf,"flightStartTime=%s",buf);
			if (succ != 0) strcpy(start_time,strip(buf));
         succ = sscanf(buf,"latitude=%s>",buf);
         if (succ != 0) {
            strcpy(long_lat_coords[coord_ind],strip(buf));
            coord_ind++;
         }
         succ = sscanf(buf,"longitude=%[^/>]s>",buf);
         if (succ != 0) {
            strcpy(long_lat_coords[coord_ind],strip(buf));
            coord_ind++;
         }
		}

      result.start_time = start_time;
      result.end_time = end_time;
      result.num_coords = static_cast<int>(coord_ind/2);
      for(int i  = 0; i <coord_ind;i+=2){
         result.long_lat_coords[i][0] = atof(long_lat_coords[i]);
         result.long_lat_coords[i][1] = atof(long_lat_coords[i+1]);
      }
   fclose(fp);
   return result;
}


using namespace time_literals;

static constexpr unsigned max_mandatory_gyro_count = 1;
static constexpr unsigned max_optional_gyro_count = 4;
static constexpr unsigned max_mandatory_accel_count = 1;
static constexpr unsigned max_optional_accel_count = 4;
static constexpr unsigned max_mandatory_mag_count = 1;
static constexpr unsigned max_optional_mag_count = 4;
static constexpr unsigned max_mandatory_baro_count = 1;
static constexpr unsigned max_optional_baro_count = 4;

bool PreFlightCheck::preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				    vehicle_status_flags_s &status_flags, const bool checkGNSS, bool report_failures, const bool prearm,
				    const hrt_abstime &time_since_boot)
{
	report_failures = (report_failures && status_flags.condition_system_hotplug_timeout
			   && !status_flags.condition_calibration_enabled);

        FILE * file;
        int vehi_gps_pos=orb_subscribe(ORB_ID(vehicle_gps_position));
        time_t timestamp;
	struct vehicle_gps_position_s raw;
	orb_copy(ORB_ID(vehicle_gps_position),vehi_gps_pos,&raw);
	timestamp=(raw.time_utc_usec)/1000000;//microsecond to seconds
	struct tm  ts;
        Date_time DT;//current time
        ts = *localtime(&timestamp);
        DT.Year=ts.tm_year+1900;
        DT.Month=ts.tm_mon+1;
        DT.date=ts.tm_mday;
        DT.Hours=ts.tm_hour;
        DT.Minutes=ts.tm_min;
        DT.Seconds=ts.tm_sec;
	        printf("\n%d/%d/%d current %d:%d:%d \n",DT.Year,DT.Month,DT.date,DT.Hours,DT.Minutes,DT.Seconds);
		//printf("\ntimestamp::::%lu\n",raw.time_utc_usec);


	file = fopen("./log/permission_artifact_breach.xml", "r");// ./log/ for posix /log/ for nuttx
	if (file){
	//printf("file exists\n");
	fclose(file);
	GEO_DATE_TIME_XML Rv;
	ParsedData data= parse_artifact();

        char *p,*q;
	p=data.start_time;
	q=data.end_time;
         char df[4];
         memcpy(df,p,4);
        printf("oye  %s",df);
        Rv.start.Year=atoi(df);
        char mf[2];
        memcpy(mf,(p+5),2);
	Rv.start.Month=atoi(mf)/10000;
	memcpy(mf,(p+8),2);
	Rv.start.date=atoi(mf)/10000;
	memcpy(df,q,4);
	Rv.end.Year=atoi(df);
	memcpy(mf,(q+5),2);
	Rv.end.Month=atoi(mf)/10000;
	memcpy(mf,(q+8),2);
	Rv.end.date=atoi(mf)/10000;
	memcpy(mf,(p+11),2);
	Rv.start.Hours=atoi(mf)/10000;
	memcpy(mf,(p+14),2);
	Rv.start.Minutes=atoi(mf)/10000;
	memcpy(mf,(p+17),2);
	Rv.start.Seconds=atoi(mf)/10000;
	memcpy(mf,(q+11),2);
	Rv.end.Hours=atoi(mf)/10000;
	memcpy(mf,(q+14),2);
	Rv.end.Minutes=atoi(mf)/10000;
	memcpy(mf,(q+17),2);
	Rv.end.Seconds=atoi(mf)/10000;

        printf("\nfrom xml:: %d/%d/%d and  %d:%d:%d",Rv.end.Year,Rv.end.Month,Rv.end.date,Rv.end.Hours,Rv.end.Minutes,Rv.end.Seconds);


	bool in_time=In_Time(DT,Rv);
	if (in_time!=1){
		printf("\n permission denied\n");
		return false;
	}
	(void) data;

/*
	Add code comparing data.start_time and data.end_time with current local time.
	Return false if time requirement is not satisfied, otherwise continue with
	other checks
	Also add code to check if certificate is valid, otherwise return false
*/
	int has_artifact = 1;
	param_set(param_find("PERM_ARTIFACT"),&has_artifact);
	printf("Permission Artifact Found \n");
	}

	else{
   	//file doesn't exists or cannot be opened (es. you don't have access permission)
	int has_artifact = 0;
	param_set(param_find("PERM_ARTIFACT"),&has_artifact);
	printf("No  \n");
	return false;
	}

	int in_fence = 0;
	param_get(param_find("IN_FENCE"),&in_fence);

	if (!in_fence) return false;

	bool failed = false;

	failed = failed || !airframeCheck(mavlink_log_pub, status);

	/* ---- MAG ---- */
	{
		int32_t sys_has_mag = 1;
		param_get(param_find("SYS_HAS_MAG"), &sys_has_mag);

		if (sys_has_mag == 1) {

			/* check all sensors individually, but fail only for mandatory ones */
			for (unsigned i = 0; i < max_optional_mag_count; i++) {
				const bool required = (i < max_mandatory_mag_count) && (sys_has_mag == 1);
				bool report_fail = report_failures;

				int32_t device_id = -1;

				if (!magnetometerCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
					if (required) {
						failed = true;
					}

					report_fail = false; // only report the first failure
				}
			}

			// TODO: highest priority mag

			/* mag consistency checks (need to be performed after the individual checks) */
			if (!magConsistencyCheck(mavlink_log_pub, status, report_failures)) {
				failed = true;
			}
		}
	}

	/* ---- ACCEL ---- */
	{
		/* check all sensors individually, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_accel_count; i++) {
			const bool required = (i < max_mandatory_accel_count);
			bool report_fail = report_failures;

			int32_t device_id = -1;

			if (!accelerometerCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
				if (required) {
					failed = true;
				}

				report_fail = false; // only report the first failure
			}
		}

		// TODO: highest priority (from params)
	}

	/* ---- GYRO ---- */
	{
		/* check all sensors individually, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_gyro_count; i++) {
			const bool required = (i < max_mandatory_gyro_count);
			bool report_fail = report_failures;

			int32_t device_id = -1;

			if (!gyroCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
				if (required) {
					failed = true;
				}

				report_fail = false; // only report the first failure
			}
		}

		// TODO: highest priority (from params)
	}

	/* ---- BARO ---- */
	{
		int32_t sys_has_baro = 1;
		param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

		bool baro_fail_reported = false;

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_baro_count; i++) {
			const bool required = (i < max_mandatory_baro_count) && (sys_has_baro == 1);
			bool report_fail = (report_failures && !baro_fail_reported);

			int32_t device_id = -1;

			if (!baroCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
				if (required) {
					baro_fail_reported = true;
				}

				report_fail = false; // only report the first failure
			}
		}
	}

	/* ---- IMU CONSISTENCY ---- */
	// To be performed after the individual sensor checks have completed
	{
		if (!imuConsistencyCheck(mavlink_log_pub, status, report_failures)) {
			failed = true;
		}
	}

	/* ---- AIRSPEED ---- */
	/* Perform airspeed check only if circuit breaker is not engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || status.is_vtol)) {

		int32_t airspeed_mode = 0;
		param_get(param_find("FW_ARSP_MODE"), &airspeed_mode);
		const bool optional = (airspeed_mode == 1);

		int32_t max_airspeed_check_en = 0;
		param_get(param_find("COM_ARM_ARSP_EN"), &max_airspeed_check_en);

		float airspeed_stall = 10.0f;
		param_get(param_find("ASPD_STALL"), &airspeed_stall);

		const float arming_max_airspeed_allowed = airspeed_stall / 2.0f; // set to half of stall speed

		if (!airspeedCheck(mavlink_log_pub, status, optional, report_failures, prearm, (bool)max_airspeed_check_en,
				   arming_max_airspeed_allowed)
		    && !(bool)optional) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT) {
		if (rcCalibrationCheck(mavlink_log_pub, report_failures, status.is_vtol) != OK) {
			if (report_failures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}

			failed = true;

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true, false, status);
			status_flags.rc_calibration_valid = false;

		} else {
			// The calibration is fine, but only set the overall health state to true if the signal is not currently lost
			status_flags.rc_calibration_valid = true;
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true,
					 !status.rc_signal_lost, status);
		}
	}

	/* ---- SYSTEM POWER ---- */
	if (status_flags.condition_power_input_valid && !status_flags.circuit_breaker_engaged_power_check) {
		if (!powerCheck(mavlink_log_pub, status, report_failures, prearm)) {
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	// only check EKF2 data if EKF2 is selected as the estimator and GNSS checking is enabled
	int32_t estimator_type = -1;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
		param_get(param_find("SYS_MC_EST_GROUP"), &estimator_type);

	} else {
		// EKF2 is currently the only supported option for FW & VTOL
		estimator_type = 2;
	}

	if (estimator_type == 2) {
		// don't report ekf failures for the first 10 seconds to allow time for the filter to start
		bool report_ekf_fail = (time_since_boot > 10_s);

		if (!ekf2Check(mavlink_log_pub, status, false, report_failures && report_ekf_fail, checkGNSS)) {
			failed = true;
		}

		if (!ekf2CheckSensorBias(mavlink_log_pub, report_failures && report_ekf_fail)) {
			failed = true;
		}
	}

	/* ---- Failure Detector ---- */
	if (!failureDetectorCheck(mavlink_log_pub, status, report_failures, prearm)) {
		failed = true;
	}

	failed = failed || !manualControlCheck(mavlink_log_pub, report_failures);
	failed = failed || !cpuResourceCheck(mavlink_log_pub, report_failures);

	/* Report status */
	return !failed;
}

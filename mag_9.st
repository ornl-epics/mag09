/*
SNL Program for mag_9 superconducting magnet.

This uses one Oxford Mercury-iPS power supply. This SNL acts as a top level
interface to the oxford power supply and provide the sequencing logic
involved in ramping up and down the magnet.

Macros required:
S - Base PV name for the SNL template that provides a top level interface.
PS - Base PV name of the Oxford

Based on the Matt Pearson's Mag_b superconducting magnet with the Oxford Instruments IPS 120
June 2019

There is a hard coded 10s timeout for the pvPut with callback. So
to handle longer timeouts we have to use pvPutComplete and 
pvPut with ASYNC option, rather than pvPut with SYNC option. 
We then wait for the completion in the next state.

States:

If any of the below states time out, we go to ERROR state. 

INIT:  Executed at startup. Does some initial checks, then goes to IDLE state.
IDLE:  This is the state from which we can start a ramp. I checks that the field 
       is zero (on the power supply) and the persistent switch heater (HEATER) is off.
PSU_RAMP: Ramp up the controller to match the persistent magnetic field (if not zero).
          This state waits for the controller current to match the last current reading
          when the HEATER was switched off last.
HEATER_ON: Set HEATER on for the controller. Wait for the controller to report 'on'.
MAGNET_RAMP: Set a new field on the controller. Wait until the copntroller report success (+/- 0.12T)
HEATER_OFF: Set HEATER off on the controller.
PSU_ZERO: Set current to zero on the controller. Wait until it's close to zero.
END: Goes to IDLE if everything is OK, or goes to ERROR if a problem is detected.
ERROR: The error state. To get out of this state a user reset must
       be performed.
RESET: Executed after an ERROR state, and returned the state
       machine to IDLE.

*/

program mag_9

option +r;
option +d;

%{#include <string.h>}%
%{#include <stdlib.h>}%
%{#include <unistd.h>}%
%{#include <math.h>}%

/* General defines */
#define START_ 1
#define STOP_ 0
#define OK_ 0
#define ERROR_ 1

#define DEBUG_L0 0
#define DEBUG_L1 1
#define DEBUG_L2 2

#define STATE_INIT_        0x01
#define STATE_IDLE_        0x02
#define STATE_PSU_RAMP_    0x04
#define STATE_HEATER_ON_   0x08
#define STATE_MAGNET_RAMP_ 0x10
#define STATE_HEATER_OFF_  0x20
#define STATE_PSU_ZERO_    0x40
#define STATE_END_         0x80
#define STATE_ERROR_       0x100
#define STATE_RESET_       0x200
#define STATE_OSC_         0x400
#define STATE_WIG_         0x800

#define MAXCHAR 1024

#define HEATER_OFF_ 0            /*defining the persistence switch states*/ 
#define HEATER_ON_ 1
#define HEATER_STS_WAITING_ 2

#define ACTIVITY_HOLD_ 0      
#define ACTIVITY_SET_ 1
#define ACTIVITY_ZERO_ 2
#define ACTIVITY_CLAMP_ 3

#define RAMP_STAT_NONE_ 0
#define RAMP_STAT_INTER_ 1
#define RAMP_STAT_FINAL_ 2
#define RAMP_STAT_FINISHED_ 3

#define F_DeadBand_ 0.005     /*Constant used for deadband of retries of the Field SP*/
#define I_DeadBand_ 0.005     /*Constang used for deadband of retries of the current SP*/
#define MAX_CMD_RETRIES 20     /*This is the number of packet resends before we quit */ 

unsigned short heater_pv;
unsigned short heaterSet_pv;
unsigned short heateri;       /*Heater counter for waiting*/ 

unsigned short rampStatus_pv;

float fieldSetpointRead_pv;     

float fieldSetpointSet_pv;
float rampingTo_pv;
float fieldDeadband_pv;
float field_pv; 
float fieldSet_pv;
float lastField_pv;
float lastTargetField_pv;
float current_pv;
float currentDeadband_pv;
unsigned short activityRead_pv;
unsigned short activitySet_pv;
unsigned short ramp_pv;
unsigned short stopRamp_pv;
unsigned short reset_pv;
unsigned short state_pv;
unsigned short error_pv;
string message_pv;
unsigned short wasreset_pv;
unsigned short ramping_pv;
unsigned short debug_pv;
unsigned short wait_pv;
unsigned short waitHeater_pv;
float timeout_pv;
unsigned short commandRetries; 
unsigned short skip_ramp; 
unsigned short timerClear_pv;
unsigned short timerControl_pv;
unsigned short delay1sec; 


assign heater_pv to "{PS}:SwitchTSts";                          /*read the switch after timer*/ 
assign heaterSet_pv to "{PS}:SET_SWHT"; 			/*set the switch1 on|off*/
assign fieldSetpointRead_pv to "{PS}:READ_FSET";  	/*read the field setpoint*/
assign fieldSetpointSet_pv to "{PS}:SET_FSET";         /*set the field setpoint*/
assign field_pv to "{PS}:READ_FLD";                      /*read the actual field*/
assign lastField_pv to "{PS}:READ_PFLD";        /*persistent field readback*/
assign current_pv to "{PS}:READ_CURR";                  /*read the actual current*/
assign activityRead_pv to "{PS}:READ_ACTN";
assign activitySet_pv to "{PS}:SET_ACTN";
assign fieldSet_pv to "{S}:FieldSet";   			/*set the field*/
assign rampingTo_pv to "{S}:RampingTo";
assign fieldDeadband_pv to "{S}:FieldDeadband";   
assign currentDeadband_pv to "{S}:CurrentDeadband";
assign ramp_pv to "{S}:Ramp";
assign stopRamp_pv to "{S}:StopRamp";
assign reset_pv to "{S}:Reset";
assign state_pv to "{S}:State";
assign error_pv to "{S}:Error";
assign message_pv to "{S}:Message";
assign wasreset_pv to "{S}:WasReset";
assign ramping_pv to "{S}:Ramping";
assign debug_pv to "{S}:DebugLevel";
assign wait_pv to "{S}:WaitTime";
assign waitHeater_pv to "{S}:WaitTimeHeater";
assign timeout_pv to "{S}:Timeout";                      
assign lastTargetField_pv to "{S}:LastTargetField"; /*Used to keep track of a multi stage ramp.*/
assign rampStatus_pv to "{S}:RampStatus";
assign timerClear_pv to "{S}:ClearTimer.PROC";
assign timerControl_pv to "{S}:Timer.DISA";


monitor fieldSetpointRead_pv;
monitor activityRead_pv;
monitor timeout_pv;
monitor ramp_pv;
monitor stopRamp_pv;
monitor reset_pv;
monitor heater_pv;
monitor field_pv;
monitor current_pv;
monitor currentDeadband_pv;
monitor fieldDeadband_pv;
monitor fieldSet_pv;

#define OSC_RAMP_0_ 0
#define OSC_RAMP_DOWN_ 1
#define OSC_RAMP_UP_ 2
#define OSC_RAMP_SP_ 3
#define OSC_RAMP_DONE_ 4

#define OSC_WAIT_T1_ 90
#define OSC_WAIT_T2_ 30



/* Local variables */
short local_debug;


/* States */
entry
{
   printf("##################runControl: SNL running.\n");
}


ss mag_9{


    /******************************************/
    /* INIT STATE */
    /*INIT:  Executed at startup. Does some initial checks, then goes to IDLE state.*/	
    /******************************************/
    state init {
        entry {

            printf("############mag_9: SNL init.!!!!!!!!!!!!!!!!!!!!\n");

            state_pv = STATE_INIT_; 
            pvPut(state_pv, SYNC);
	    error_pv = OK_;
	    pvPut(error_pv, SYNC);
	    ramping_pv = STOP_;
	    pvPut(ramping_pv, SYNC);
            rampStatus_pv = RAMP_STAT_NONE_;
            pvPut(rampStatus_pv, SYNC);
	    strcpy(message_pv, "mag_9 SNL Started OK");
	    pvPut(message_pv);
        }
        
	when() {
	    printf("mag_9: SNL init body.\n");
	} state idle
	
        exit {
            printf("mag_9: SNL init exit.\n");
        }
    } 


    /******************************************/
    /* IDLE STATE */
    /*IDLE:  This is the state from which we can start a ramp. I checks that the field 
       is zero (on the power supply) and the persistent switch heater (HEATER) is off.*/
    /******************************************/
    state idle {

        entry {
	    if (local_debug > DEBUG_L1) {
               printf("mag_9: idle state.\n");
	    }
	    printf("mag_9: idle state.\n");
	    state_pv = STATE_IDLE_;
	    pvPut(state_pv, SYNC);
	    strcpy(message_pv, "Idle");
	    pvPut(message_pv, SYNC);
	    ramping_pv = 0;
	    pvPut(ramping_pv, SYNC);
	    rampingTo_pv = 0;
	    pvPut(rampingTo_pv, SYNC);
	    stopRamp_pv = 0;
	    pvPut(stopRamp_pv);
	    /* Set ramp_pv = STOP_ to do callback on busy record.*/
	    ramp_pv = 0;
	    pvPut(ramp_pv);
            rampStatus_pv = RAMP_STAT_NONE_;
            pvPut(rampStatus_pv, SYNC);
            skip_ramp = 0;
            /* Set the field deadband equal to the standard 0.001T */
            fieldDeadband_pv = 0.001;
            pvPut(fieldDeadband_pv, SYNC);
            timerControl_pv = 1;
            pvPut(timerControl_pv, SYNC);
            timerClear_pv = 1;
            pvPut(timerClear_pv);

        }

        /* When we detect a user start. */
        when (ramp_pv == 1) {
	     pvGet(debug_pv, SYNC);
	     local_debug = debug_pv;

	     if (local_debug > DEBUG_L0) {
       		printf("mag_9: idle state: Debug level for this ramp: %d\n", local_debug);
 	     }

	     strcpy(message_pv, "Starting Ramp Process");
 	     pvPut(message_pv, SYNC);

	     /* Set ramping flag */
	     if (ramp_pv == 1) {
	     	ramping_pv = START_;
	     	pvPut(ramping_pv, SYNC);
	     }
	     stopRamp_pv = STOP_;
	     pvPut(stopRamp_pv);

	     /* Clear the Reset and WasReset record*/
	     wasreset_pv = OK_;
	     pvPut(wasreset_pv, SYNC);
	     reset_pv = STOP_;
	     pvPut(reset_pv, SYNC);

	     /* Read the wait time and timeout times */
 	     pvGet(wait_pv, SYNC);
	     pvGet(timeout_pv, SYNC); 

	     epicsThreadSleep(1);

	     /* Check the HEATER state on the PS. If its not off, set the error flag.*/
	     if (heater_pv != HEATER_OFF_) {
 	        strcpy(message_pv, "Cannot Ramp If HEATER IS On or waiting");
		pvPut(message_pv, SYNC);
        	error_pv = ERROR_;
        	pvPut(error_pv, SYNC);
        	if (local_debug > DEBUG_L0) {
                  printf("mag_9: idle state: Cannot ramp if the HEATER is not off. Aborting.\n");
        	}
              }	    

        } state psu_ramp
	
    }

    /******************************************/
    /* PSU_RAMP STATE */
    /*PSU_RAMP: Ramp up the controller to match the persistent magnetic field (if not zero).
          This state waits for the controller current to match the last current reading
          when the HEATER was switched off last.*/
    /******************************************/
    state psu_ramp {
       option -t;  
       entry {
          epicsThreadSleep(wait_pv);
          if (local_debug > DEBUG_L1) {
	     printf("mag_9: psu_ramp state.\n");
	  }
	  state_pv = STATE_PSU_RAMP_;
	  pvPut(state_pv, SYNC);
	  
          if (error_pv != ERROR_) {
             if (error_pv == OK_) {  
	        strcpy(message_pv, "Ramping Power Supply");
	        pvPut(message_pv, SYNC);
	     }

	     /* Read old field from the power supply.*/
	     pvGet(lastField_pv, SYNC);

             pvGet(fieldSet_pv, SYNC);
	     rampingTo_pv = fieldSet_pv;
             pvPut(rampingTo_pv, SYNC);

             lastTargetField_pv = lastField_pv;
             pvPut(lastTargetField_pv, SYNC);
             fieldSetpointSet_pv = lastField_pv;
             
             /* Set the setpoint before we do a GOTO SET */
             commandRetries = 0;
             while (error_pv != ERROR_) {
                pvPut(fieldSetpointSet_pv, SYNC);   
                epicsThreadSleep(wait_pv); 
                pvGet(fieldSetpointRead_pv, SYNC);
                if (fieldSetpointSet_pv != fieldSetpointRead_pv) {
                   commandRetries++;
                   if (local_debug > DEBUG_L0) {
	              printf("mag_9: psu_ramp state: fieldSetpointSet try %d fieldSetpointSet_pv %f fieldSetpointRead_pv %f fieldSet_pv %f\n", commandRetries, fieldSetpointSet_pv, fieldSetpointRead_pv, fieldSet_pv);
	           }
                   if (commandRetries >= MAX_CMD_RETRIES) {
                      strcpy(message_pv, "Failed To Set Setpoint When Ramping Power Supply");
	              pvPut(message_pv, SYNC);
                      error_pv = ERROR_;
	              pvPut(error_pv, SYNC);
                      printf("mag_9: failed to set setpoint %d\n");

			}
                } else {
                   break;
                }
             }    

             epicsThreadSleep(wait_pv);
             
             /* Do a GOTO SET */
             activitySet_pv = ACTIVITY_SET_;
             commandRetries = 0;
             while (error_pv != ERROR_) {
                pvPut(activitySet_pv, SYNC);
                epicsThreadSleep(0.5);			       
                pvGet(activityRead_pv, SYNC);
                if (activityRead_pv != activitySet_pv) {
                   commandRetries++;
                   epicsThreadSleep(wait_pv);
                   if (local_debug > DEBUG_L0) {
	              printf("mag_b: psu_ramp state: activitySet try!!!!! %d, ActivityRead %d, ActivitySet %d\n", commandRetries, activityRead_pv, activitySet_pv);
	           }
                   if (commandRetries >= MAX_CMD_RETRIES) {
                      strcpy(message_pv, "Failed To GOTO SET When Ramping Power Supply");
	              pvPut(message_pv, SYNC);
                      error_pv = ERROR_;
	              pvPut(error_pv, SYNC);
                   }
                } else {
                   break;
                }
             }       
          }
       }


     /* Go to error state if the error PV is set*/
     when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: psu_ramp state: Going to error state.\n");
          }
     } state error

     when (delay(timeout_pv)) {
    	   strcpy(message_pv, "PSU Ramp Timeout.");
    	   pvPut(message_pv, SYNC);
    	   error_pv = ERROR_;
    	   pvPut(error_pv, SYNC);
    	   if (local_debug > DEBUG_L0) {
    		   printf("mag_9: psu_ramp state: PSU Ramp Timeout. lastField=%f.\n", lastField_pv);
    	   }
      } state error

      
      /* Wait for power supply to reach required field */
      when ((field_pv > (lastField_pv - fieldDeadband_pv)) && (field_pv < (lastField_pv + fieldDeadband_pv))) {
       	  if (local_debug > DEBUG_L0) {
             printf("mag_bg psu_ramp state: power supply field reached expected value. lastField=%f. field=%f.\n", lastField_pv, field_pv);
	  }
      } state HEATER_on    

   }

   /******************************************/
    /* HEATER_ON STATE */
    /*HEATER_ON: Set HEATER on for the controller. Wait for the controller to report 'on'.*/
    /******************************************/
    state HEATER_on {
       option -t; 
       
       entry {
    	   epicsThreadSleep(wait_pv);
    	   commandRetries = 0; 
    	   if (local_debug > DEBUG_L1) {
    	      printf("mag_9: HEATER_on state.\n");
    	   }
         
    	   state_pv = STATE_HEATER_ON_;
    	   pvPut(state_pv, SYNC);
	  
    	   if (error_pv == OK_) {
    	      strcpy(message_pv, "Turning On HEATER");
    	      pvPut(message_pv, SYNC);	     
 
              /* Turn on heater switch */
              heaterSet_pv = HEATER_ON_;
              commandRetries = 0;
              while (error_pv != ERROR_) {
              	 pvPut(heaterSet_pv, SYNC);
                 epicsThreadSleep(wait_pv);
		 pvGet(heater_pv, SYNC);
		 if (commandRetries == 5)
	            {  
			/*delay for heater status check of Mag-09*/ 
	                heateri=0; 
			while ((heateri<200)&&(heater_pv != heaterSet_pv)){
				heateri++;
				strcpy(message_pv, "Waiting for HEATER ON");
          	       	 	pvPut(message_pv, SYNC);
				printf("waiting for heater timer %d Sec\n", 200-heateri);
	                	epicsThreadSleep(1);
			}
	            }		
		 if (heater_pv != heaterSet_pv) {
              	    commandRetries++;
		    if (local_debug > DEBUG_L0) {
		       printf("mag_9: heater_on state: heaterSet try %d\n", commandRetries);
		    }
              	    if (commandRetries >= MAX_CMD_RETRIES) {
              	       strcpy(message_pv, "Failed To Turn On Heater Switch");
	      	       pvPut(message_pv, SYNC);
		       error_pv = ERROR_;
		       pvPut(error_pv, SYNC);
              	    }
              	 } else {
              	   break;
              	 }
              }
              strcpy(message_pv, "Waiting For Heater Switch");
	      pvPut(message_pv, SYNC);

              /* Enable timer */ 
              timerControl_pv = 0;
              pvPut(timerControl_pv, SYNC);
              timerClear_pv = 1;
              pvPut(timerClear_pv);   

	      epicsThreadSleep(waitHeater_pv); 

              /* Disable timer */ 
              timerControl_pv = 1;
              pvPut(timerControl_pv, SYNC);
              timerClear_pv = 1;
              pvPut(timerClear_pv); 

    	   }

       }

       /* Go to error state if the error PV is set*/
       when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: HEATER_on state: Going to error state.\n");
          }
       } state error

       when (delay(timeout_pv)) {
    	   strcpy(message_pv, "HEATER On Timeout");
    	   pvPut(message_pv, SYNC);
    	   error_pv = ERROR_;
    	   pvPut(error_pv, SYNC);
    	   if (local_debug > DEBUG_L0) {
    	      printf("mag_9: HEATER_on state: HEATER On Timeout.\n");
    	   }
       } state error

       when (stopRamp_pv == START_) {
    	   strcpy(message_pv, "Ramp Aborted. Skipping This Step.");
    	   pvPut(message_pv, SYNC);
    	   printf("mag_9: HEATER_on state: Magnet Ramp Manually Aborted.\n");
       } state psu_zero  
  
	when (heater_pv == HEATER_ON_ && ramp_pv == 1) {
             if (local_debug > DEBUG_L0) {
        	printf("mag_9: HEATER_on state: HEATER is on.\n");
             }
        } state magnet_ramp   

	
   }

    /******************************************/
    /* MAGNET_RAMP STATE */
    /*MAGNET_RAMP: Set a new field on the controller. Wait until the copntroller report success (+/- 0.12T)*/
    /******************************************/
    state magnet_ramp {
       option -te;
       entry {
    	  epicsThreadSleep(wait_pv);          
          if (local_debug > DEBUG_L1) {
             printf("mag_9: magnet_ramp state.\n");
          }

          state_pv = STATE_MAGNET_RAMP_;
          pvPut(state_pv, SYNC);

          if (rampStatus_pv == RAMP_STAT_NONE_) {
             rampStatus_pv = RAMP_STAT_INTER_;
          }

	  if ((error_pv == OK_) && (rampStatus_pv != RAMP_STAT_FINISHED_)) {
             strcpy(message_pv, "Ramping Magnet");
             pvPut(message_pv, SYNC);

             if (stopRamp_pv !=  START_) {
             	/* Read desired new field and set on power supply*/
        	pvGet(fieldSet_pv, SYNC);
        	if (local_debug > DEBUG_L0) {
        	   printf("mag_9: magnet_ramp state: New desired field: %fT\n", fieldSet_pv);
        	}

		fieldSetpointSet_pv = fieldSet_pv;
	        rampingTo_pv = fieldSet_pv;
                rampStatus_pv = RAMP_STAT_FINAL_;
                lastTargetField_pv = fieldSetpointSet_pv;
                pvPut(lastTargetField_pv, SYNC);
            

                /* Set the field setpoint on the controller */
                commandRetries = 0;
                while (error_pv != ERROR_) {
                   pvPut(fieldSetpointSet_pv, SYNC);
                   epicsThreadSleep(wait_pv);
		   pvGet(fieldSetpointRead_pv, SYNC);
                   if (fieldSetpointRead_pv != fieldSetpointSet_pv) {
                      commandRetries++;
                      if (local_debug > DEBUG_L0) {
	                 printf("mag_9: magnet_ramp state: fieldSetpointSet_pv try %d\n", commandRetries);
	              }
                      if (commandRetries >= MAX_CMD_RETRIES) {
                         strcpy(message_pv, "Failed To Set Field Setpoint");
	                 pvPut(message_pv, SYNC);
                         error_pv = ERROR_;
	                 pvPut(error_pv, SYNC);
                      }
                   } else {
                      break;
                   }
                }
    
                pvPut(rampingTo_pv, SYNC);
                pvPut(rampStatus_pv, SYNC);
                if (local_debug > DEBUG_L0) {
                   printf("mag_9: magnet_ramp state: ramping to: %fT\n", fieldSet_pv);
                } 

                /* Do a GOTO SET */
                activitySet_pv = ACTIVITY_SET_;
                commandRetries = 0;
                while (error_pv != ERROR_) {
                   pvPut(activitySet_pv, SYNC);
                   epicsThreadSleep(0.5);       
                   pvGet(activityRead_pv, SYNC);
                   if (activityRead_pv != activitySet_pv) {
                     commandRetries++;
                     epicsThreadSleep(wait_pv);
                      if (local_debug > DEBUG_L0) {
	                printf("mag_9: magnet_ramp state: activitySet try %d , Read %d, Set %d \n", commandRetries, activityRead_pv, activitySet_pv);
	              }
                      if (commandRetries >= MAX_CMD_RETRIES) {
                        strcpy(message_pv, "Failed To GOTO SET When Ramping Magnet");
	                pvPut(message_pv, SYNC);
                        error_pv = ERROR_;
	                pvPut(error_pv, SYNC);
                      }
                   } else {
                      /* Enable timer for the start of the ramp */ 
                      timerControl_pv = 0;
                      pvPut(timerControl_pv, SYNC);
                      timerClear_pv = 1;
                      pvPut(timerClear_pv); 
                      break;
                   }
                } 
             }  
          }
       }
       
       /* Go to error state if the error PV is set*/
       when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: magnet_ramp state: Going to error state.\n");
          }
       } state error

       when (delay(timeout_pv)) {
          strcpy(message_pv, "Magnet Ramp Timeout.");
          pvPut(message_pv, SYNC);
          error_pv = ERROR_;
          pvPut(error_pv, SYNC);
          printf("mag_9: magnet_ramp state: Magnet Ramp Timeout. field: %fT \n", field_pv);
       } state error
       
       when (stopRamp_pv == START_) {
          strcpy(message_pv, "Active Ramp Aborted. Sending HOLD.");
          pvPut(message_pv, SYNC);
          printf("mag_9: magnet_ramp state: magnet ramp manually aborted. sending HOLD command and finishing as normal.\n");
	  rampStatus_pv = RAMP_STAT_FINISHED_;
	  pvPut(rampStatus_pv, SYNC);
          stopRamp_pv = STOP_;
          pvPut(stopRamp_pv, SYNC);
       } state magnet_ramp

       when (rampStatus_pv == RAMP_STAT_FINISHED_) {

          /* Do a HOLD */
	  /* Wait for Stabilization*/
          epicsThreadSleep(wait_pv);
          activitySet_pv = ACTIVITY_HOLD_;
          commandRetries = 0;
          while (error_pv != ERROR_) {
             pvPut(activitySet_pv, SYNC);
             epicsThreadSleep(0.5);       
             pvGet(activityRead_pv, SYNC);
             if (activityRead_pv != activitySet_pv) {
                commandRetries++;
                epicsThreadSleep(wait_pv);
                if (local_debug > DEBUG_L0) {
	           printf("mag_9: magnet_ramp state: activitySet HOLD try %d\n", commandRetries);
	        }
                if (commandRetries >= MAX_CMD_RETRIES) {
                   strcpy(message_pv, "Failed To HOLD At End Of Ramp");
	           pvPut(message_pv, SYNC);
                      error_pv = ERROR_;
	              pvPut(error_pv, SYNC);
                   }
             } else {
                break;
             }
          }
           /*stabilization*/

        strcpy(message_pv, "Stabilization before heateroff");
        pvPut(message_pv, SYNC);

        epicsThreadSleep(wait_pv*20);
	   
 
       } state HEATER_off

        when ((field_pv > (fieldSetpointSet_pv - fieldDeadband_pv)) && (field_pv < (fieldSetpointSet_pv + fieldDeadband_pv))) {
	/*stabilization possible problem*/   
        strcpy(message_pv, "Stabilization after within tol");
        pvPut(message_pv, SYNC);
	epicsThreadSleep(wait_pv*20);	

       	   if (local_debug > DEBUG_L0) {
              printf("mag_9: magnet_ramp state: power supply fields reached expected value. field: %fT, fieldSetpointSet: %fT, field deadband: %f\n", field_pv, fieldSetpointSet_pv, fieldDeadband_pv);
	  }
          if (rampStatus_pv == RAMP_STAT_FINAL_) {
             rampStatus_pv = RAMP_STAT_FINISHED_;
             pvPut(rampStatus_pv, SYNC);

             /* Disable timer for the end of the ramp */ 
             timerControl_pv = 1;
             pvPut(timerControl_pv, SYNC);
             timerClear_pv = 1;
             pvPut(timerClear_pv); 
          }
       } state magnet_ramp

   }



    /******************************************/
    /* HEATER_OFF STATE */
    /*HEATER_OFF: Set HEATER off on the controller.*/
    /******************************************/
    state HEATER_off {
       option -t; 
       entry {
	/*wait for longer stabiliation time*/
	  epicsThreadSleep(wait_pv*6);
          if (local_debug > DEBUG_L1) {
             printf("mag_9: heater_off state.\n");
	  }
           
	  state_pv = STATE_HEATER_OFF_;
	  pvPut(state_pv, SYNC);

	  strcpy(message_pv, "Turning Off Heater Switch");
	  pvPut(message_pv, SYNC);
          
          /* Turn off heater switch */
          heaterSet_pv = HEATER_OFF_;
          commandRetries = 0;
          while (error_pv != ERROR_) {
             pvPut(heaterSet_pv, SYNC);
             epicsThreadSleep(wait_pv);
             pvGet(heater_pv, SYNC);

                 if (commandRetries == 5)
                    {
                        /*delay for heater status check of Mag-09*/
                        heateri=0;
                        while ((heateri<200)&&(heater_pv != heaterSet_pv)){
                                heateri++;
                                strcpy(message_pv, "Waiting for HEATER OFF");
                                pvPut(message_pv, SYNC);
                                printf("waiting for heater timer %d Sec\n", 200-heateri);
                                epicsThreadSleep(1);
                        }
                    }


             if ((heater_pv != heaterSet_pv)) {
                commandRetries++;
	        printf("mag_9: heater_off state: heaterSet try %d\n", commandRetries);
                if (commandRetries >= MAX_CMD_RETRIES) {
                   strcpy(message_pv, "Failed To Turn Off Heater Switch");
	           pvPut(message_pv, SYNC);
                   error_pv = ERROR_;
	           pvPut(error_pv, SYNC);
                }
             } else {
                break;
             }
          }      
          strcpy(message_pv, "Waiting For Heater Switch");
	  pvPut(message_pv, SYNC);

          /* Enable timer */ 
          timerControl_pv = 0;
          pvPut(timerControl_pv, SYNC);
          timerClear_pv = 1;
          pvPut(timerClear_pv); 
 
          epicsThreadSleep(waitHeater_pv);

          /* Disable timer */ 
          timerControl_pv = 1;
          pvPut(timerControl_pv, SYNC);
          timerClear_pv = 1;
          pvPut(timerClear_pv);

        }

       /* Go to error state if the error PV is set*/
       when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: HEATER_off state: Going to error state.\n");
	  }
       } state error

       when (delay(timeout_pv)) {
          strcpy(message_pv, "HEATER Off Timeout");
	  pvPut(message_pv, SYNC);
	  error_pv = ERROR_;
	  pvPut(error_pv, SYNC);
          if (local_debug > DEBUG_L0) {
             printf("mag_9: HEATER_off state: HEATER Off Timeout.\n");
	  }
       } state error


       when ((heater_pv == HEATER_OFF_)) {
       	  if (local_debug > DEBUG_L0) {
             printf("mag_9: HEATER_off state: HEATER is off.\n");
	  }
       } state psu_zero   
   
       when ((heater_pv == HEATER_OFF_ )) {
       	  if (local_debug > DEBUG_L0) {
             printf("mag_9: Wiggle & HEATER_off state: HEATER is off. Wait 90s.\n");
	  }
          epicsThreadSleep(90);

       } state psu_zero
    }


    /******************************************/
    /* PSU_ZERO STATE */
    /*PSU_ZERO: Set current to zero on the controller. Wait until it's close to zero.*/
    /******************************************/
    state psu_zero {
       option -t;
       entry {
          epicsThreadSleep(wait_pv); 
          if (local_debug > DEBUG_L1) {
	     printf("mag_9: psu_zero state.\n");
	  }
          rampingTo_pv = 0;
          pvPut(rampingTo_pv, SYNC);

	  state_pv = STATE_PSU_ZERO_;
	  pvPut(state_pv, SYNC);

	  if (error_pv == OK_) {
	     strcpy(message_pv, "Setting Power Supply Current To Zero");
	     pvPut(message_pv, SYNC);

             /* Do a GOTO ZERO */
             activitySet_pv = ACTIVITY_ZERO_;
             commandRetries = 0;
             while (error_pv != ERROR_) {
                pvPut(activitySet_pv, SYNC);
                epicsThreadSleep(0.5);       
                pvGet(activityRead_pv, SYNC);
                printf("mag_9: psu_zero state retry.\n");

                if (activityRead_pv != activitySet_pv) {
                   	commandRetries++;
                   	epicsThreadSleep(5);
	           	printf("mag_9: psu_zero state: activitySet try %d\n", commandRetries);
                   	if (commandRetries >= MAX_CMD_RETRIES) {
                      		strcpy(message_pv, "Failed To GOTO ZERO");
	              		pvPut(message_pv, SYNC);
                      		error_pv = ERROR_;
	              		pvPut(error_pv, SYNC);
                   	}
                } 
		else {
        	        /* Enable timer  */ 
                        timerControl_pv = 0;
                   	pvPut(timerControl_pv, SYNC);
                        timerClear_pv = 1;
                        pvPut(timerClear_pv); 
                        break;
                     }
             }  
	  }
       }

       /* Go to error state if the error PV is set*/
       when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: psu_zero state: Going to error state.\n");
	  }
       } state error

       when (delay(timeout_pv)) {
       	  strcpy(message_pv, "Timeout. Current Did Not Reach Zero.");
	  pvPut(message_pv, SYNC);
	  error_pv = ERROR_;
	  pvPut(error_pv, SYNC);
          if (local_debug > DEBUG_L0) {
             printf("mag_9: psu_zero state: Current did not reach zero. current1: %fA\n", current_pv);
	  }
       } state error


       /* Wait for the supply to reach zero current */
       when (current_pv < currentDeadband_pv) {
          printf("mag_9: psu_zero state: Power supply current reached zero. current: %fA\n", current_pv);	     

          /* Disable timer  */ 
          timerControl_pv = 1;
          pvPut(timerControl_pv, SYNC);
          timerClear_pv = 1;
          pvPut(timerClear_pv); 
        
       } state end

    }
    
    /*END: Goes to IDLE if everything is OK, or goes to ERROR if a problem is detected.*/
    /******************************************/
    state end {
        
       entry {
             if (local_debug > DEBUG_L1) {
             printf("mag_9: end state.\n");
          }
          state_pv = STATE_END_; 
          pvPut(state_pv, SYNC);
       }

       /* Go to error state if the error PV is set*/
       when (error_pv == ERROR_) {
          if (local_debug > DEBUG_L0) {
             printf("mag_9: end state: Going to error state.\n");
	  }
       } state error

       when () {
          epicsThreadSleep(1);  
       } state idle

       exit {
          if (local_debug > DEBUG_L1) {
             printf("mag_9: end state. exit.\n");
          }
       }
    }


    /******************************************/
    /* ERROR STATE */
    /*ERROR: The error state. To get out of this state a user reset must be performed.*/
    /******************************************/
    state error {
       entry {
          if (local_debug > DEBUG_L1) {
             printf("mag_9: error state.\n");
	  }
          printf("mag_9: error state.\n");
  	  state_pv = STATE_ERROR_; 
          pvPut(state_pv, SYNC);
	  /*Set busy record back to zero*/
	  ramp_pv = STOP_;	
	  pvPut(ramp_pv);
	  stopRamp_pv = STOP_;	
	  pvPut(stopRamp_pv);
	  ramping_pv = STOP_;	
	  pvPut(ramping_pv);
          /* Disable timer  */ 
          timerControl_pv = 1;
          pvPut(timerControl_pv, SYNC);
          timerClear_pv = 1;
          pvPut(timerClear_pv); 

       }

       /* Wait for a user reset */
       when (reset_pv == START_) {
          if (local_debug > DEBUG_L1) {
             printf("mag_9: error state: Going to reset state.\n");
	  }       
       } state reset

    }

    /******************************************/
    /* RESET STATE */
    /*RESET: Executed after an ERROR state, and returned the state machine to IDLE.*/
    /******************************************/
    state reset {
       entry {
          if (local_debug > DEBUG_L1) {
             printf("mag_9: reset state.\n");
	  }
	  state_pv = STATE_RESET_; 
          pvPut(state_pv, SYNC);
	  /*Set the WasReset record*/
	  wasreset_pv = ERROR_;
	  pvPut(wasreset_pv, SYNC);
       }

       when () {	    
	  /* Clear error and reset flags */
	  reset_pv = STOP_;
	  pvPut(reset_pv, SYNC);
	  error_pv = OK_;
	  pvPut(error_pv, SYNC);
	  ramp_pv = STOP_;
	  pvPut(ramp_pv);
	  stopRamp_pv = STOP_;	
	  pvPut(stopRamp_pv);
	  ramping_pv = STOP_;
	  pvPut(ramping_pv);
       } state idle
    
    }
}





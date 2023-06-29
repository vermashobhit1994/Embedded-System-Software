/* program to parse gps packet format using state machine

  Approach used: break input string into substrings.
                 find out from each substring which to be put into output string.
				 put into output string.
				 
	NOTE: This program has been tested for memory leak with drmemory.exe			 
*/


/************************************************** header files ********************************************************************************/
#include <stdio.h>
#include <stdlib.h> //for calloc()
#include <string.h>
/*************************************************************************************************************************************************/


/************************************************ preprocessor macros *****************************************************************************/
#define GPS_SENTENCE_FIXED_DATA_TYPE "$GPGGA"
#define GPS_SENTENCE_REAL_DATA_TYPE  "$GPRMC"

#define  OUTPUT_STRING_FIELDS_LEN (15)

#define OUTPUT_STRING_LEN_MAX (100)
/*************************************************************************************************************************************************/


/************************************************************** Global variables *************************************************************/  
char** parseString = NULL;

char* outputString = NULL;

unsigned char GPRMC_packet_found = 0;
unsigned char GPGGA_packet_found = 0;	
/***********************************************************************************************************************************************/


/************************************************ Enum to indicate output string indexes ********************************************************/
//anonymous enum for output string 
enum
{
	OUTPUT_STRING_INDEX_DATE = 0,
	OUTPUT_STRING_INDEX_UTC_POSITION_FIX,
	OUTPUT_STRING_INDEX_LATITUDE,
	OUTPUT_STRING_INDEX_LATITUDE_DIRECTION,
	OUTPUT_STRING_INDEX_LONGITUDE,
	OUTPUT_STRING_INDEX_LONGITUDE_DIRECTION,
	OUTPUT_STRING_INDEX_GPS_QUALITY,
	OUTPUT_STRING_INDEX_NO_OF_SV,
	OUTPUT_STRING_INDEX_HDOP,
	OUTPUT_STRING_INDEX_SPEED_OVER_GROUND,
	OUTPUT_STRING_INDEX_TRACK_ANGLE,
	OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT,
	OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT_UNIT,
	OUTPUT_STRING_INDEX_GEOID_SEPARATION,
	OUTPUT_STRING_INDEX_GEOID_SEPARATION_UNIT
};
/***************************************************************************************************************************************************/



/******************************* Documentation Section *********************************************
 * @fn                - parse_input_string 
 
 * @brief             - private function used to create output packet as array of pointers to strings
 *                       using input packet type.
 
 * @input[param1]     - input string containing GPGGA or GPRMC or both packet type.
 
 * @return            - none 
 
 * @note              - each field is checked in GPGGA or GPRMC or both and then placed at proper index 
 *                      at output string.
 *                     Making all fields in parse string to be null terminated.
 ****************************************************************************************************/
static void parse_input_string(char* inputString)
{
    char* separator  = ",*" ;

	//allocate memory for number of fields in parse string 	to fields in output string 
    parseString = calloc(OUTPUT_STRING_FIELDS_LEN, sizeof(char*) );
        

	//store gps packet type
    char* token = strtok(inputString, separator); 
    
	
	while( token != NULL)
    {
        //found GPRMC packet type
        if (strncmp(token, GPS_SENTENCE_REAL_DATA_TYPE, strlen(GPS_SENTENCE_REAL_DATA_TYPE)) == 0)
        {
			GPRMC_packet_found = 1;
			
            //Step1: put the utc at index 1
            token = strtok(NULL, separator); //skip packet type and get utc
            parseString[OUTPUT_STRING_INDEX_UTC_POSITION_FIX] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_UTC_POSITION_FIX], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_UTC_POSITION_FIX][strlen(token)] = '\0';
            
			


            //Step2: skip status
            token = strtok(NULL, separator); 
          
            //Step3: put latitude at index 2
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_LATITUDE] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_LATITUDE], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_LATITUDE][strlen(token)] = '\0';
            
			
		  
            //Step4: put direction of latitude at index 3
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_LATITUDE_DIRECTION] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_LATITUDE_DIRECTION], token, strlen(token));
            parseString[OUTPUT_STRING_INDEX_LATITUDE_DIRECTION][strlen(token)] = '\0';
            
            
			
            //Step5: put longitude at index 4
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_LONGITUDE] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_LONGITUDE], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_LONGITUDE][strlen(token)] = '\0';
            
			
 
            //Step6: put direction of longitude at index 5
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_LONGITUDE_DIRECTION] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_LONGITUDE_DIRECTION], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_LONGITUDE_DIRECTION][strlen(token)] = '\0';
            

 
            //Step7: put speed at index 9
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_SPEED_OVER_GROUND] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_SPEED_OVER_GROUND], token, strlen(token));
            parseString[OUTPUT_STRING_INDEX_SPEED_OVER_GROUND][strlen(token)] = '\0';
               
			   
            //Step8: put track angle at index 10
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_TRACK_ANGLE] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_TRACK_ANGLE], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_TRACK_ANGLE][strlen(token)] = '\0';
            

            //Step9: put date at index 0
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_DATE] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_DATE], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_DATE][strlen(token)] = '\0';
            

            //step10: skip magnetic variation
            token = strtok(NULL, separator);
           
            //step11: skip checksum
            token = strtok(NULL, separator);            
           
		    //step12: go to next string after checksum
            token = strtok(NULL, separator);

            
           
        }
        else if(strncmp(token, GPS_SENTENCE_FIXED_DATA_TYPE, strlen(GPS_SENTENCE_FIXED_DATA_TYPE)) == 0)
        {
			GPGGA_packet_found = 1;
            
            //Step1: skip utc
            token = strtok(NULL, separator);

            //Step2: skip latitude
            token = strtok(NULL, separator);
 
            //Step3: skip direction of latitude
            token = strtok(NULL, separator);
     
            //Step4: skip longitude
            token = strtok(NULL, separator);
 
            //Step5: skip direction of longitude
            token = strtok(NULL, separator);
                        
            //Step6: put gps at 6th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_GPS_QUALITY] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_GPS_QUALITY], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_GPS_QUALITY][strlen(token)] = '\0';

            //Step7: put no of sv at 7th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_NO_OF_SV] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_NO_OF_SV], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_NO_OF_SV][strlen(token)] = '\0';
            
            
            //Step8: put hdop at 8th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_HDOP] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_HDOP], token, strlen(token));
            parseString[OUTPUT_STRING_INDEX_HDOP][strlen(token)] = '\0';
            

            //Step9: put orthometric height at 11th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT][strlen(token)] = '\0';
            
            //Step10: put orthometric height unit at 12th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT_UNIT] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT_UNIT], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_ORTHOMETRIC_HEIGHT_UNIT][strlen(token)] = '\0';
            

            //Step11: put geoid separation at 13th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION][strlen(token)] = '\0';
            
			
            //Step12: put geoid separation unit at 14th index
            token = strtok(NULL, separator);
            parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION_UNIT] = calloc(strlen(token)+1, sizeof(char) );
            strncpy(parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION_UNIT], token, strlen(token));
			parseString[OUTPUT_STRING_INDEX_GEOID_SEPARATION_UNIT][strlen(token)] = '\0';
            

            //Step13: skip age of gps
            token  = strtok(NULL, separator);
 
            //Step14: skip station id
            token = strtok(NULL, separator);

            //Step15: skip checksum
            token = strtok(NULL, separator);
 
			//go to next string after checksum  
			token = strtok(NULL, separator);
  

        }
 
        
    }
		
	
 
}



	


/******************************* Documentation Section *********************************************
 * @fn                - parse_gps_string 
 * @brief             - parse input string and then store result into output string 
 * @input[param1]     - input string containing GPGGA or GPRMC or both packet type. 
 * @return            -  output string with desired output fields.
 * @note              -  
 ****************************************************************************************************/

char* parse_gps_string(char* string)
{

	/************************ Step1: parse input string *******************************************************************/
	//NOTE: Here break given string into tokens and then put desired token order into "parseString" 
    parse_input_string(string);
	/**********************************************************************************************************************/


	
	//pointer copy so that original pointer location don't change
    char* copyOutputString = outputString;
    	
	char* outputStringSeparator = "," ;
	
	int j=0;
	char** tempString  = NULL;
	char* token = NULL;
	
	
	/******************************* Step2: break output string into array of pointers to string ************************************/
	                               //NOTE: this is only done if not filling characters in output string for first time.
	 
	token = strtok(copyOutputString, outputStringSeparator);
	
	//if entering data into output string other than first time 
	if (token != NULL)
	{
		tempString = calloc(OUTPUT_STRING_FIELDS_LEN, sizeof(char*));
		
		//copy all data from original output string to temporary string 
		while(token )
		{
			
			tempString[j] = calloc(strlen(token)+1, sizeof(char) );
			
			
			strncpy(tempString[j], token, strlen(token)+1);
			j++;
			
			token = strtok(NULL, outputStringSeparator);
			
		}
		//j point to last filled location due to security reasons
		j--;
		 
	/************************************************************************************************************************************/
	
		
	/****************************************** Step3: copy desired data into array of pointers to string ********************************/
	                                            //NOTE: +1 is done while copying data for copying of '\0' value  
	 
		/*************************** Both packet type present ********************************************************/
		if (GPRMC_packet_found == 1 && GPGGA_packet_found == 1)
		{	
			/* copy all fields from parseString to temporary string by first deleting old data of temporary string.
			   this is done forcefully so that if both fields are present but any data inside GPGGA packet 
			   or GPRMC packet gets changed then we get new data .		
			*/
			for(int i=0; i < OUTPUT_STRING_FIELDS_LEN; i++)
			{
				//copy data from parse string to temporary string if string exists in parse string 
				if (parseString[i] != NULL)
				{
					//delete old data 
					free(tempString[i]);
					tempString[i] = NULL;
					
					//allocate memory for new data 
					tempString[i] = calloc( strlen(parseString[i])+1, sizeof(char));
										
					//copy new data 							
					strncpy(tempString[i], parseString[i], strlen(parseString[i]) +1);
				}
				 		
			}
		}
		/*************************************************************************************************************/
		
		/*************************** Only GPRMC packet type present ********************************************************/
		else if (GPRMC_packet_found == 1 && GPGGA_packet_found == 0)
		{
			
			
			for(int i=0; i < OUTPUT_STRING_FIELDS_LEN; i++)
			{
				//copy indexes i.e. 0, 1, 2, 3, 4,5, 9, 10 from parseString to tempString since it indicates GPRMC fields 
				if ( (i>=0 && i<= 5) || i==9 || i==10)
				{
					//copy data from parse string to temporary string if string exists in parse string 	
					if (parseString[i] != NULL)
					{
						//delete old data
						free(tempString[i]);
						tempString[i] = NULL; //avoid dangling pointer
						
						//allocate memory for new data
						tempString[i] = calloc( strlen(parseString[i])+1, sizeof(char));
						
						//copy new data
						strncpy(tempString[i], parseString[i], strlen(parseString[i]) +1);
						
						
					}
				}
				
				//delete data at indexes 6,7,8,11,12,13,14 i.e. GPGGA field data 
				else if ((i >=6 && i<=8) || (i>=11 && i<=14))
				{
					//delete GPGGA field data in output string 	
					free(tempString[i]);
					tempString[i] = NULL; //avoid dangling pointer 
				}
	
			}
		}
		/*************************************************************************************************************/
		
		
		/*************************** Only GPGGA packet type present ********************************************************/
		                             //NOTE: data from index 1 to 5 remains unchanged
									 
		else if (GPRMC_packet_found == 0 && GPGGA_packet_found == 1)
		{
			for(int i=0; i < OUTPUT_STRING_FIELDS_LEN; i++)
			{
				//copy indexes i.e. 6,7,8,11, 12, 13, 14 from parseString
				if ( (i>=6 && i<= 8) || (i>=11 && i<=14) )
				{
					
					//copy data from parse string to temporary string if string exists in parse string 
					if (parseString[i] != NULL)
					{
						//delete old data
						free(tempString[i]);
						tempString[i] = NULL; //avoid dangling pointer
						
						//allocate memory for new data
						tempString[i] = calloc( strlen(parseString[i])+1, sizeof(char));
						
						//copy new data
						strncpy(tempString[i], parseString[i], strlen(parseString[i])+1 );
					}					
						
				}
				//delete data at indexes 0,9,10 of GPRMC field data 
				else if ( i==0 || i==9 || i== 10)
				{
					//delete GPRMC field data in output string
					free(tempString[i]);
					tempString[i] = NULL;  //avoid dangling pointer
				}		
			}
		}
		/*************************************************************************************************************/
		
	/************************************************************************************************************************************/
		
		
	}
	else
	{
		
		/**************************** Step5: copy data from parse string to output string **************************************************/   
			//when inserting elements for first time
			for(int i=0; i < OUTPUT_STRING_FIELDS_LEN; i++)
			{
				if (parseString[i] != NULL)
				{
					//copy field data from parseString to output String 	
					strncpy(copyOutputString, parseString[i], strlen(parseString[i]) );
					copyOutputString += strlen(parseString[i]);  //move pointer to next unfilled location 
				}
				//don't put ',' at last field of output string 
				if ( i != OUTPUT_STRING_FIELDS_LEN-1)
				{
					strncpy(copyOutputString,outputStringSeparator, strlen(outputStringSeparator) );
					copyOutputString += strlen(outputStringSeparator); //move pointer to next unfilled location 
				}
				
			}
		/***************************************************************************************************************************************/
	}
	
	
	
		
	/******************************************* Step 7: Convert back data from array of pointers to string to comma separated string ****************************/ 	
	//data entered in output string other than first time  
	if (j > 0)
	{
		//reset pointer 
		copyOutputString = outputString;
		
		/********************************************* Reset output String ************************************************/
		//memset(copyOutputString, 0, strlen(copyOutputString) );
		for(int j=0; j < OUTPUT_STRING_LEN_MAX;j++)
			copyOutputString[j] = '0';
		/*****************************************************************************************************************/
		
		
		int i=0;
		int countFilledCharacters = 0; //store actual number of filled characters in output string 
		
		for(i=0; i<OUTPUT_STRING_FIELDS_LEN;i++)
		{
			//copy temporary string field only if it exists 
			if (tempString[i] != NULL)
			{
				//copy from temporary string to output string 
				strncpy(copyOutputString, tempString[i], strlen(tempString[i]) );
				copyOutputString += strlen(tempString[i]);//move pointer to next unfilled location 
			}
			
			//don't put ',' at last field of output string
			//copy ','	
			if ( i != OUTPUT_STRING_FIELDS_LEN-1)
			{
				strncpy(copyOutputString, outputStringSeparator, strlen(outputStringSeparator) );
				copyOutputString += strlen(outputStringSeparator); //move pointer to next unfilled location 
			
			}	
			
		    	
		}
		countFilledCharacters = copyOutputString-outputString; 
		
		//make output characters a string 
		outputString[countFilledCharacters] = '\0';
		
		/**********************************************************************************************************************************************/
		
		/***************************************************** Step8: free memory for temporary string  ************************************************/
		//free memory allocated to temporary string 
		for(int i=0; i<OUTPUT_STRING_FIELDS_LEN;i++)
		{
			free(tempString[i]) ; 
			tempString[i] = NULL;
		}
		free(tempString);
		tempString = NULL;
		/**********************************************************************************************************************************************/
		
		
	}
		
	/************************************************** Step9: reset packet found flag **************************************************************/
                                                   //this is done so that flag set based on packet present in input string for next input.	
	//reset flags
	GPRMC_packet_found =0;
	GPGGA_packet_found = 0;
	
	/**********************************************************************************************************************************************/
	
	
	/***************************************************** Step10: free memory for parse string  ************************************************/
	
	//free memory for parse string array
    for(int i=0; i<OUTPUT_STRING_FIELDS_LEN;i++)
	{
		free(parseString[i]);
		parseString[i] = NULL;
	}
	
	free(parseString);
	parseString = NULL;
	/**********************************************************************************************************************************************/
	

   
    return outputString;       
}


char* outString = NULL;


/***************** NOTE: here test cases are written separately for various inputs so  that order of them can be tested **************************/
void test_case_first(void)
{
	char inputString_first[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,*6A,"
"$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-"
"25.669,M,2.0,0031*4F";


	printf("input string \n%s\n", inputString_first);
	outString = parse_gps_string(inputString_first);
    printf("output string \n %s\n\n\n", outString);
    
}


void test_case_second(void)
{
	char inputString_second[] = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-"
"25.669,M,2.0,0031*4F,$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230"
"394,003.1,*6A";
    printf("input string \n%s\n", inputString_second);
	
	outString = parse_gps_string(inputString_second);
	
	printf("output string \n %s\n\n\n", outString);
    
}


void test_case_third(void)
{
	
	char inputString_third[] = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-"
"25.669,M,2.0,0031*4F";
    printf("input string \n%s\n", inputString_third);
	
	outString = parse_gps_string(inputString_third);

	printf("output string \n %s\n\n\n", outString);
    	

}



void test_case_fourth(void)
{
	char inputString_fourth[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,*6A";
	printf("input string \n%s\n", inputString_fourth);
	
	outString = parse_gps_string(inputString_fourth);
	printf("output string \n %s\n\n\n", outString);
    
}


void test_pattern_first(void)
{
	
	//first order for inputs 
    test_case_third();		
	test_case_fourth();	
	test_case_first();
	test_case_second();	
	
}



int main()
{
	
	//allocate memory dynamically to output string that can store when all fields are present in output string
	outputString = calloc(OUTPUT_STRING_LEN_MAX, sizeof(char) ); //+1 for '\0'
		
	

	
	//second order of inputs
	test_case_first();
	test_case_second();
	test_case_third();
	test_case_fourth();
	
	//free memory for output string
    free(outputString);
    outputString = NULL;

}
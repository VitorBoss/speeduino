/*
 * This routine is used for doing any data conversions that are required during firmware changes
 * This prevents users getting difference reports in TS when such a data change occurs.
 * It also can be used for setting good values when there are viarables that move locations in the ini
 * When a user skips multiple firmware versions at a time, this will roll through the updates 1 at a time
 */
#include "globals.h"
#include "storage.h"
#include EEPROM_LIB_H //This is defined in the board .h files

void doUpdates()
{
  #define CURRENT_DATA_VERSION    17
  //Only the latest updat for small flash devices must be retained
   #ifndef SMALL_FLASH_MODE

  //May 2017 firmware introduced a -40 offset on the ignition table. Update that table to +40
  if(EEPROM.read(EEPROM_DATA_VERSION) == 2)
  {
    for(int x=0; x<16; x++)
    {
      for(int y=0; y<16; y++)
      {
        ignitionTable.values[x][y] = ignitionTable.values[x][y] + 40;
      }
    }
    writeAllConfig();
    //EEPROM.write(EEPROM_DATA_VERSION, 3);
    storeEEPROMVersion(3);
  }
  //June 2017 required the forced addition of some CAN values to avoid weird errors
  if(EEPROM.read(EEPROM_DATA_VERSION) == 3)
  {
    configPage9.speeduino_tsCanId = 0;
    configPage9.true_address = 256;
    configPage9.realtime_base_address = 336;

    //There was a bad value in the May base tune for the spark duration setting, fix it here if it's a problem
    if(configPage4.sparkDur == 255) { configPage4.sparkDur = 10; }

    writeAllConfig();
    //EEPROM.write(EEPROM_DATA_VERSION, 4);
    storeEEPROMVersion(4);
  }
  //July 2017 adds a cranking enrichment curve in place of the single value. This converts that single value to the curve
  if(EEPROM.read(EEPROM_DATA_VERSION) == 4)
  {
    //Some default values for the bins (Doesn't matter too much here as the values against them will all be identical)
    configPage10.crankingEnrichBins[0] = 0;
    configPage10.crankingEnrichBins[1] = 40;
    configPage10.crankingEnrichBins[2] = 70;
    configPage10.crankingEnrichBins[3] = 100;

    configPage10.crankingEnrichValues[0] = 100 + configPage2.crankingPct;
    configPage10.crankingEnrichValues[1] = 100 + configPage2.crankingPct;
    configPage10.crankingEnrichValues[2] = 100 + configPage2.crankingPct;
    configPage10.crankingEnrichValues[3] = 100 + configPage2.crankingPct;

    writeAllConfig();
    //EEPROM.write(EEPROM_DATA_VERSION, 5);
    storeEEPROMVersion(5);
  }
  //September 2017 had a major change to increase the minimum table size to 128. This required multiple pieces of data being moved around
  if(EEPROM.read(EEPROM_DATA_VERSION) == 5)
  {
    //Data after page 4 has to move back 128 bytes
    for(int x=0; x < 1152; x++)
    {
      int endMem = EEPROM_CONFIG10_END - x;
      int startMem = endMem - 128; //
      byte currentVal = EEPROM.read(startMem);
      EEPROM.update(endMem, currentVal);
    }
    //The remaining data only has to move back 64 bytes
    for(int x=0; x < 352; x++)
    {
      int endMem = EEPROM_CONFIG10_END - 1152 - x;
      int startMem = endMem - 64; //
      byte currentVal = EEPROM.read(startMem);
      EEPROM.update(endMem, currentVal);
    }

    EEPROM.write(EEPROM_DATA_VERSION, 6);
    loadConfig(); //Reload the config after changing everything in EEPROM
  }
  //November 2017 added the staging table that comes after boost and vvt in the EEPROM. This required multiple pieces of data being moved around
  if(EEPROM.read(EEPROM_DATA_VERSION) == 6)
  {
    //Data after page 8 has to move back 82 bytes
    for(int x=0; x < 529; x++)
    {
      int endMem = EEPROM_CONFIG10_END - x;
      int startMem = endMem - 82; //
      byte currentVal = EEPROM.read(startMem);
      EEPROM.update(endMem, currentVal);
    }

    EEPROM.write(EEPROM_DATA_VERSION, 7);
    loadConfig(); //Reload the config after changing everything in EEPROM
  }

  if (EEPROM.read(EEPROM_DATA_VERSION) == 7) {
    //Convert whatever flex fuel settings are there into the new tables

    configPage10.flexBoostBins[0] = 0;
    configPage10.flexBoostAdj[0]  = (int8_t)configPage2.aeColdPct;

    configPage10.flexFuelBins[0] = 0;
    configPage10.flexFuelAdj[0]  = configPage2.idleUpPin;

    configPage10.flexAdvBins[0] = 0;
    configPage10.flexAdvAdj[0]  = configPage2.aeTaperMin;

    for (uint8_t x = 1; x < 6; x++)
    {
      uint8_t pct = x * 20;
      configPage10.flexBoostBins[x] = pct;
      configPage10.flexFuelBins[x] = pct;
      configPage10.flexAdvBins[x] = pct;

      int16_t boostAdder = (((configPage2.aeColdTaperMin - (int8_t)configPage2.aeColdPct) * pct) / 100) + (int8_t)configPage2.aeColdPct;
      configPage10.flexBoostAdj[x] = boostAdder;

      uint8_t fuelAdder = (((configPage2.idleUpAdder - configPage2.idleUpPin) * pct) / 100) + configPage2.idleUpPin;
      configPage10.flexFuelAdj[x] = fuelAdder;

      uint8_t advanceAdder = (((configPage2.aeTaperMax - configPage2.aeTaperMin) * pct) / 100) + configPage2.aeTaperMin;
      configPage10.flexAdvAdj[x] = advanceAdder;
    }

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 8);
  }

  if (EEPROM.read(EEPROM_DATA_VERSION) == 8)
  {
    //May 2018 adds separate load sources for fuel and ignition. Copy the existing load alogirthm into Both
    configPage2.fuelAlgorithm = configPage2.legacyMAP; //Was configPage2.unused2_38c
    configPage2.ignAlgorithm = configPage2.legacyMAP; //Was configPage2.unused2_38c

    //Add option back in for open or closed loop boost. For all current configs to use closed
    configPage4.boostType = 1;

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 9);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 9)
  {
    //October 2018 set default values for all the aux in variables (These were introduced in Aug, but no defaults were set then)
    //All aux channels set to Off
    for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
    {
      configPage9.caninput_sel[AuxinChan] = 0;
    }

    //Ability to change the analog filter values was added. Set default values for these:
    configPage4.ADCFILTER_TPS = 50;
    configPage4.ADCFILTER_CLT = 180;
    configPage4.ADCFILTER_IAT = 180;
    configPage4.ADCFILTER_O2  = 128;
    configPage4.ADCFILTER_BAT = 128;
    configPage4.ADCFILTER_MAP = 20;
    configPage4.ADCFILTER_BARO= 64;

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 10);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 10)
  {
    //May 2019 version adds the use of a 2D table for the priming pulse rather than a single value.
    //This sets all the values in the 2D table to be the same as the previous single value
    configPage2.primePulse[0] = configPage2.aeColdTaperMax / 5; //New priming pulse values are in the range 0-127.5 rather than 0-25.5 so they must be divided by 5
    configPage2.primePulse[1] = configPage2.aeColdTaperMax / 5; //New priming pulse values are in the range 0-127.5 rather than 0-25.5 so they must be divided by 5
    configPage2.primePulse[2] = configPage2.aeColdTaperMax / 5; //New priming pulse values are in the range 0-127.5 rather than 0-25.5 so they must be divided by 5
    configPage2.primePulse[3] = configPage2.aeColdTaperMax / 5; //New priming pulse values are in the range 0-127.5 rather than 0-25.5 so they must be divided by 5
    //Set some sane default temperatures for this table
    configPage2.primeBins[0] = 0;
    configPage2.primeBins[1] = 40;
    configPage2.primeBins[2] = 70;
    configPage2.primeBins[3] = 100;

    //Also added is coolant based ASE for both duration and amount
    //All the adder amounts are set to what the single value was previously
    configPage2.asePct[0] = configPage2.aeColdTaperMin;
    configPage2.asePct[1] = configPage2.aeColdTaperMin;
    configPage2.asePct[2] = configPage2.aeColdTaperMin;
    configPage2.asePct[3] = configPage2.aeColdTaperMin;
    //ASE duration is set to 10s for all coolant values
    configPage2.aseCount[0] = 10;
    configPage2.aseCount[1] = 10;
    configPage2.aseCount[2] = 10;
    configPage2.aseCount[3] = 10;
    //Finally the coolant bins for the above are set to sane values (Rememerbing these are offset values)
    configPage2.aseBins[0] = 0;
    configPage2.aseBins[1] = 20;
    configPage2.aseBins[2] = 60;
    configPage2.aseBins[3] = 80;

    //Coolant based ignition advance was added also. Set sane values
    configPage4.cltAdvBins[0] = 0;
    configPage4.cltAdvBins[1] = 30;
    configPage4.cltAdvBins[2] = 60;
    configPage4.cltAdvBins[3] = 70;
    configPage4.cltAdvBins[4] = 85;
    configPage4.cltAdvBins[5] = 100;
    configPage4.cltAdvValues[0] = 0;
    configPage4.cltAdvValues[1] = 0;
    configPage4.cltAdvValues[2] = 0;
    configPage4.cltAdvValues[3] = 0;
    configPage4.cltAdvValues[4] = 0;
    configPage4.cltAdvValues[5] = 0;


    //March 19 added a tacho pulse duration that could default to stupidly high values. Check if this is the case and fix it if found. 6ms is tha maximum allowed value
    if(configPage2.tachoDuration > 6) { configPage2.tachoDuration = 3; }

    //MAP based AE was introduced, force the AE mode to be TPS for all existing tunes
    configPage2.aeMode = AE_MODE_TPS;
    configPage2.maeThresh = configPage2.taeThresh;
    //Set some sane values for the MAP AE curve
    configPage4.maeRates[0] = 75;
    configPage4.maeRates[1] = 75;
    configPage4.maeRates[2] = 75;
    configPage4.maeRates[3] = 75;
    configPage4.maeBins[0] = 7;
    configPage4.maeBins[1] = 12;
    configPage4.maeBins[2] = 20;
    configPage4.maeBins[3] = 40;

    //The 2nd fuel table was added. To prevent issues, force it to be disabled.
    configPage10.fuel2Mode = 0;


    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 11);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 11)
  {
    //Sep 2019
    //A battery calibration offset value was introduced. Set default value to 0
    configPage4.batVoltCorrect = 0;

    //An option was added to select the older method of performing MAP reads with the pullup resistor active
    configPage2.legacyMAP = 0;

    //Secondary fuel table was added for swtiching. Make sure it's all turned off initially
    configPage10.fuel2Mode = 0;
    configPage10.fuel2SwitchVariable = 0; //Set switch variable to RPM
    configPage10.fuel2SwitchValue = 7000; //7000 RPM switch point is safe

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 12);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 12)
  {
    //Nov 2019
    //New option to only apply voltage correction to dead time. Set existing tunes to use old method
    configPage2.battVCorMode = BATTV_COR_MODE_WHOLE;

    //Manual baro correction curve was added. Give it some default values (All baro readings set to 100%)
    configPage4.baroFuelBins[0] = 80;
    configPage4.baroFuelBins[1] = 85;
    configPage4.baroFuelBins[2] = 90;
    configPage4.baroFuelBins[3] = 95;
    configPage4.baroFuelBins[4] = 100;
    configPage4.baroFuelBins[5] = 105;
    configPage4.baroFuelBins[6] = 110;
    configPage4.baroFuelBins[7] = 115;

    configPage4.baroFuelValues[0] = 100;
    configPage4.baroFuelValues[1] = 100;
    configPage4.baroFuelValues[2] = 100;
    configPage4.baroFuelValues[3] = 100;
    configPage4.baroFuelValues[4] = 100;
    configPage4.baroFuelValues[5] = 100;
    configPage4.baroFuelValues[6] = 100;
    configPage4.baroFuelValues[7] = 100;

    //Idle advance curve was added. Add default values
    configPage2.idleAdvEnabled = 0; //Turn this off by default
    configPage2.idleAdvTPS = 5; //Active below 5% tps
    configPage2.idleAdvRPM = 20; //Active below 2000 RPM
    configPage4.idleAdvBins[0] = 30;
    configPage4.idleAdvBins[1] = 40;
    configPage4.idleAdvBins[2] = 50;
    configPage4.idleAdvBins[3] = 60;
    configPage4.idleAdvBins[4] = 70;
    configPage4.idleAdvBins[5] = 80;
    configPage4.idleAdvValues[0] = 15; //These values offset by 15, so this is just making this equal to 0
    configPage4.idleAdvValues[1] = 15;
    configPage4.idleAdvValues[2] = 15;
    configPage4.idleAdvValues[3] = 15;
    configPage4.idleAdvValues[4] = 15;
    configPage4.idleAdvValues[5] = 15;

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 13);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 13)
  {
    //202005
    //Cranking enrichment range 0..1275% instead of older 0.255, so need to divide old values by 5
    configPage10.crankingEnrichValues[0] = configPage10.crankingEnrichValues[0] / 5;
    configPage10.crankingEnrichValues[1] = configPage10.crankingEnrichValues[1] / 5;
    configPage10.crankingEnrichValues[2] = configPage10.crankingEnrichValues[2] / 5;
    configPage10.crankingEnrichValues[3] = configPage10.crankingEnrichValues[3] / 5;

    //Added the injector timing curve
    //Set all the values to be the same as the first one. 
    configPage2.injAng[0] = configPage2.injAng[0]; //Obviously not needed, but here for completeness
    configPage2.injAng[1] = configPage2.injAng[0];
    configPage2.injAng[2] = configPage2.injAng[0];
    configPage2.injAng[3] = configPage2.injAng[0];
    //The RPMs are divided by 100
    configPage2.injAngRPM[0] = 5;
    configPage2.injAngRPM[1] = 25;
    configPage2.injAngRPM[2] = 45;
    configPage2.injAngRPM[3] = 65;

    //Introdced a DFCO delay option. Default it to 0
    configPage2.dfcoDelay = 0;
    //Introdced a minimum temperature for DFCO. Default it to 40C
    configPage2.dfcoMinCLT = 80; //CALIBRATION_TEMPERATURE_OFFSET is 40

    //Update flexfuel ignition config values for 40 degrees offset
    for (int i=0; i<6; i++)
    {
      configPage10.flexAdvAdj[i] += 40;
    }
    
    //AE cold modifier added. Default to sane values
    configPage2.aeColdPct = 100;
    configPage2.aeColdTaperMin = 40;
    configPage2.aeColdTaperMax = 100;

    //New PID resolution, old resolution was 100% for each increase, 100% now is stored as 32
    if(configPage6.idleKP >= 8) { configPage6.idleKP = 255; }
    else { configPage6.idleKP = configPage6.idleKP<<5; }
    if(configPage6.idleKI >= 8) { configPage6.idleKI = 255; }
    else { configPage6.idleKI = configPage6.idleKI<<5; }
    if(configPage6.idleKD >= 8) { configPage6.idleKD = 255; }
    else { configPage6.idleKD = configPage6.idleKD<<5; }
    if(configPage10.vvtCLKP >= 8) { configPage10.vvtCLKP = 255; }
    else { configPage10.vvtCLKP = configPage10.vvtCLKP<<5; }
    if(configPage10.vvtCLKI >= 8) { configPage10.vvtCLKI = 255; }
    else { configPage10.vvtCLKI = configPage10.vvtCLKI<<5; }
    if(configPage10.vvtCLKD >= 8) { configPage10.vvtCLKD = 255; }
    else { configPage10.vvtCLKD = configPage10.vvtCLKD<<5; }

    //Cranking enrichment to run taper added. Default it to 0,1 secs
    configPage10.crankingEnrichTaper = 1;
    
    //ASE to run taper added. Default it to 0,1 secs
    configPage2.aseTaperTime = 1;

    // there is now optioon for fixed and relative timing retard for soft limit. This sets the soft limiter to the old fixed timing mode.
    configPage2.SoftLimitMode = SOFT_LIMIT_FIXED;

    //VSS was added for testing, disable it by default
    configPage2.vssMode = 0;

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 14);

    //
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 14)
  {
    //202008

    //MAJOR update to move the coolant, IAT and O2 calibrations to 2D tables
    int y;
    for(int x=0; x<(CALIBRATION_TABLE_SIZE/16); x++) //Each calibration table is 512 bytes long
    {
      y = EEPROM_CALIBRATION_CLT_OLD + (x * 16);
      cltCalibration_values[x] = EEPROM.read(y);
      cltCalibration_bins[x] = (x * 32);

      y = EEPROM_CALIBRATION_IAT_OLD + (x * 16);
      iatCalibration_values[x] = EEPROM.read(y);
      iatCalibration_bins[x] = (x * 32);

      y = EEPROM_CALIBRATION_O2_OLD + (x * 16);
      o2Calibration_values[x] = EEPROM.read(y);
      o2Calibration_bins[x] = (x * 32);
    }
    writeCalibration();

    //Oil and fuel pressure inputs were introduced. Disable them both by default
    configPage10.oilPressureProtEnbl = false;
    configPage10.oilPressureEnable = false;
    configPage10.fuelPressureEnable = false;

    //wmi
    configPage10.wmiEnabled = 0;
    configPage10.wmiMode = 0;
    configPage10.wmiOffset = 0;
    configPage10.wmiIndicatorEnabled = 0;
    configPage10.wmiEmptyEnabled = 0;
    configPage10.wmiAdvEnabled = 0;
    for(int i=0; i<6; i++)
    {
      configPage10.wmiAdvBins[i] = i*100/2;
      configPage10.wmiAdvAdj[i] = OFFSET_IGNITION;
    }

    //Programmable outputs added. Set all to disabled
    configPage13.outputPin[0] = 0;
    configPage13.outputPin[1] = 0;
    configPage13.outputPin[2] = 0;
    configPage13.outputPin[3] = 0;
    configPage13.outputPin[4] = 0;
    configPage13.outputPin[5] = 0;
    configPage13.outputPin[6] = 0;
    configPage13.outputPin[7] = 0;

    //New multiply MAP option added. Set new option to be the same as old
    configPage2.multiplyMAP = configPage2.multiplyMAP_old;
    //New AE option added to allow for PW added in addition to existing PW multiply
    configPage2.aeApplyMode = 0; //Set the AE mode to Multiply

    //Injector priming delay added
    configPage2.primingDelay = 0;
    //ASE taper time added
    configPage2.aseTaperTime = 10; //1 second taper

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 15);
  }

  if(EEPROM.read(EEPROM_DATA_VERSION) == 15)
  {
    //202012
    configPage10.spark2Mode = 0; //Disable 2nd spark table

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 16);
  }

  //Move this #endif to only do latest updates to safe ROM space on small devices.
  #endif
  if(EEPROM.read(EEPROM_DATA_VERSION) == 16)
  {
    //Fix for wrong placed page 13
    for(int x=EEPROM_CONFIG14_END; x>=EEPROM_CONFIG13_START; x--)
    {
      EEPROM.update(x, EEPROM.read(x-112));
    }

    configPage6.iacPWMrun = false; // just in case. This should be false anyways, but sill.
    configPage2.useDwellMap = 0; //Dwell map added, use old fixed value as default

    writeAllConfig();
    EEPROM.write(EEPROM_DATA_VERSION, 17);
  }

  //Final check is always for 255 and 0 (Brand new arduino)
  if( (EEPROM.read(EEPROM_DATA_VERSION) == 0) || (EEPROM.read(EEPROM_DATA_VERSION) == 255) )
  {
    configPage9.true_address = 0x200;
    
    //Programmable outputs added. Set all to disabled
    configPage13.outputPin[0] = 0;
    configPage13.outputPin[1] = 0;
    configPage13.outputPin[2] = 0;
    configPage13.outputPin[3] = 0;
    configPage13.outputPin[4] = 0;
    configPage13.outputPin[5] = 0;
    configPage13.outputPin[6] = 0;
    configPage13.outputPin[7] = 0;

    EEPROM.write(EEPROM_DATA_VERSION, CURRENT_DATA_VERSION);
  }

  //Check to see if someone has downgraded versions:
  if( EEPROM.read(EEPROM_DATA_VERSION) > CURRENT_DATA_VERSION ) { EEPROM.write(EEPROM_DATA_VERSION, CURRENT_DATA_VERSION); }
}

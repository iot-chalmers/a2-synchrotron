/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2017 Beshr Al Nahas and Olaf Landsiedel.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
log.log("Script started.\n");
//sim.stopSimulation();
//sim.startSimulation();  
//10min*60sec*1000ms
TIMEOUT(600000);

function SetMoteChannel(id, channel, power)
{
sim.getMoteWithID(id).getInterfaces().getRadio().setChannel(channel);
sim.getMoteWithID(id).getInterfaces().getRadio().setOutputPower(power);
}

var timeSlice = 5*60*1000*1000; //5 min in us
var step = 0;
var t0 = 0;  
/* set interferer channel */
interfererId = [7, 8];
channels = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20];
interfererPower=999999.0; //dB

/* create a log file */
date = new java.util.Date();
path = sim.getCooja().currentConfigFile.getParentFile();
logFileName = "\/log-" + sim.getTitle() + '-' + date.toString()
logFileName=logFileName.replaceAll(':', '.').replaceAll(' ', '_') +".txt";
logFilePath = path + logFileName;
outputFile = new java.io.FileWriter(logFilePath);
log.log(logFilePath+"\n");
outputFile.write(logFilePath);

/* set a new random seed */
sim.setRandomSeed(java.lang.System.currentTimeMillis());
seedStr = "New random seed: " + sim.getRandomSeedString() + "\n";
outputFile.write(seedStr);
log.log(seedStr);

while (true) {
logMsg = time + "\tID:" + id + "\t" + msg + "\n";
  //log to file
  outputFile.write(logMsg);

  //change interfered channel every 5 minutes
  if((time-t0 > step * timeSlice)) {
  	SetMoteChannel(interfererId[0], channels[step % channels.length], interfererPower);
  	t0 = time;
  	step++;
  }
  
//  log.log(logMsg);

  try{
      //This is the tricky part. The Script is terminated using
      // an exception. This needs to be caught.
      YIELD();
  } catch (e) {
      //Close the log file
      outputFile.close();
      //Rethrow exception again, to end the script.
      throw('test script finished ' + time);
  }
}


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
//5min*60sec*1000ms*1000us
timeout=5*60*1000*1000;


var timeSlice = 1*60*1000*1000; //1 min in us
var step = 0;
var t0 = 0;
globalClusterSize=4;
globalDistance=1000;

function clusterNodes(clusterSize, distance){
	count=sim.getMotesCount();
	for(i=0; i<count/clusterSize; i++){
		moveCluster(i, clusterSize, distance);
	}
}

function moveCluster(i, clusterSize, distance){
	for(j=i*clusterSize; j<i*clusterSize+clusterSize; j++){
		var mm = sim.getMoteWithID(j+1);
    var x = mm.getInterfaces().getPosition().getXCoordinate();
    var y = mm.getInterfaces().getPosition().getYCoordinate();
    var z = 0.0;
    x=i * distance;
    y=j-i*clusterSize;
		mm.getInterfaces().getPosition().setCoordinates(x, y, z);
	}
}

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


clusterNodes(globalClusterSize, globalDistance);
var startTime = time;
while (time-startTime < timeout) {
//	logMsg = time + "\tID:" + id + "\t" + msg + "\n";
//  //log to file
//  outputFile.write(logMsg);

  if((time-t0 > step * timeSlice)) {
  	
  	moveCluster(step, globalClusterSize, globalDistance);
  	t0 = time;
  	step++;
  }
  
//log.log(logMsg);
  YIELD();
}
//Close the log file
outputFile.close();
//Rethrow exception again, to end the script.
throw('test script finished ' + time);
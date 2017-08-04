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

enableDisturber = 0;

var timeSlice = 1*60*1000*1000; //5 min in us
var step = 0;
var t0 = 0;
initialPRR = 0.9;
/* set interferer channel */
channels = [11];
interferedNodes = [3, 2]
interfererPower=10.0; //dB
interfererRSSI = 10.00;
interferedPRR = 0.01; //interferer disabled -- no links
disturberId = sim.getMotesCount();

function printEdges() {   
  radioMedium = sim.getRadioMedium();
  if(radioMedium != null) {  
    radios = radioMedium.getRegisteredRadios();
    edges = radioMedium.getEdges();
    if(edges != null) {
        log.log("Edges:\n");
      for(i=0; i<edges.length; i++) {
        str = java.lang.String.format("%d -> %d, ratio: %.2f\n", new java.lang.Integer(edges[i].source.getMote().getID()), new java.lang.Integer(edges[i].superDest.radio.getMote().getID()), new java.lang.Float(edges[i].superDest.ratio));
  	    log.log(str);
      }
    } 
  }
}

function InitEdges(initialPRR, skipMote)
{
  var i=0;
  var j=0;
  var ch=0;
  var numberOfNodes = sim.getMotesCount();
  radioMedium = sim.getRadioMedium();
  if(radioMedium != null) {  
    radioMedium.clearEdges();
    for(ch=11; ch<27; ch++) {
      for(i=0; i<numberOfNodes; i++) {
      	id=i+1;
      	if(id == skipMote) {
      		continue;
      	}
      	var srcMote = sim.getMoteWithID(id);
      	if(srcMote == null) {
          str = java.lang.String.format("InitEdges: Mote %d (%d) is null!!\n", new java.lang.Integer(i+1), new java.lang.Integer(numberOfNodes));
          log.log(str);
          continue;
      	}
  	    var srcRadio = srcMote.getInterfaces().getRadio();
        for(j=0; j<numberOfNodes; j++) {
        	dstId = j+1
        	if(dstId == skipMote) {
        		continue;
        	}
        	//var ratio = myNetwork[j*numberOfNodes + i];
        	var ratio = initialPRR;
  
  	      if(i==j || ratio == 0) {
            continue;
  	      }
  	      var dstRadio = sim.getMoteWithID(dstId).getInterfaces().getRadio();
  	      var superDest = new org.contikios.cooja.radiomediums.DGRMDestinationRadio(dstRadio);
  	      superDest.ratio = ratio;
  	      superDest.channel = ch;
          var edge = new org.contikios.cooja.radiomediums.DirectedGraphMedium.Edge(srcRadio, superDest);
  	      radioMedium.addEdge(edge);
      	}
      }
    }
    
    printEdges();
    log.log("Script finished setting weights.\n"); 
  }
}

function ChangeLink(destId, channel, prr) {
	var destMote = sim.getMoteWithID(destId);
	if(destMote != null) {
  	var dstRadio = destMote.getInterfaces().getRadio();
  	var edges = radioMedium.getEdges();
  	var i;
  	for (i=0; i<edges.length; i++) {
      if (edges[i].superDest.radio == dstRadio && edges[i].superDest.channel == channel) {
        oldRatio = edges[i].superDest.ratio;
        edges[i].superDest.ratio = prr;
      }
    }
    str = java.lang.String.format("Node %d, channel %d, ratio: %.2f, was: %.2f\n", new java.lang.Integer(destId), new java.lang.Integer(channel), new java.lang.Float(prr), new java.lang.Float(oldRatio));
    log.log(str);
	} else {
    str = java.lang.String.format("Mote %d is null!!\n", new java.lang.Integer(destId));
    log.log(str);
	}
}

function CreateLink(srcId, destId, channel, prr, rssi) {
	var srcMote = sim.getMoteWithID(srcId);
	var destMote = sim.getMoteWithID(destId);
	if(srcMote != null && destMote != null) {
		var srcRadio = srcMote.getInterfaces().getRadio();
  	var dstRadio = destMote.getInterfaces().getRadio();
    var superDest = new org.contikios.cooja.radiomediums.DGRMDestinationRadio(dstRadio);
    superDest.ratio = prr;
    superDest.signal = rssi;
    superDest.channel = channel;
    var edge = new org.contikios.cooja.radiomediums.DirectedGraphMedium.Edge(srcRadio, superDest);
    radioMedium.addEdge(edge);
    str = java.lang.String.format("%d -> %d, channel: %d ratio: %.2f\n", new java.lang.Integer(srcId), new java.lang.Integer(destId), new java.lang.Integer(channel), new java.lang.Float(superDest.ratio));
    log.log(str);
	}
}

function SetMoteChannel(id, channel, power)
{
	var srcMote = sim.getMoteWithID(id);
	srcMote.getInterfaces().getRadio().setChannel(channel);
	srcMote.getInterfaces().getRadio().setOutputPower(power);
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

SetMoteChannel(disturberId, 100, 0);

InitEdges(initialPRR, disturberId);

var startTime = time;

while (time-startTime < timeout) {
	logMsg = time + "\tID:" + id + "\t" + msg + "\n";
  //log to file
  outputFile.write(logMsg);

  //change interfered channel every 5 minutes
  if((time-t0 > step * timeSlice) && enableDisturber) {
  	//SetMoteChannel(interfererId[0], channels[step % channels.length], interfererPower);
		SetMoteChannel(disturberId, channels[step % channels.length], interfererPower);
  	var i;
  	for(i=0; i<interferedNodes.length; i++) {
  		//ChangeLink(interferedNodes[i], channels[step % channels.length], interferedPRR);
  		/* Create a link form the interferer to the destination */
  		CreateLink(disturberId, interferedNodes[i], channels[step % channels.length], 1.0 - interferedPRR, interfererRSSI);
  	}
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
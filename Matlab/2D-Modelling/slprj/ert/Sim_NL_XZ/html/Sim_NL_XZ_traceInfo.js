function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Integrator */
	this.urlHashMap["Sim_NL_XZ:3"] = "Sim_NL_XZ.c:81,103,167&Sim_NL_XZ.h:46,51,56";
	/* <Root>/MATLAB Function */
	this.urlHashMap["Sim_NL_XZ:4"] = "Sim_NL_XZ.c:109&Sim_NL_XZ.h:37";
	/* <Root>/Sum */
	this.urlHashMap["Sim_NL_XZ:17"] = "Sim_NL_XZ.c:138&Sim_NL_XZ.h:39";
	/* <Root>/phi_y_dot_plot */
	this.urlHashMap["Sim_NL_XZ:5"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_XZ:5";
	/* <Root>/phi_y_plot */
	this.urlHashMap["Sim_NL_XZ:6"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_XZ:6";
	/* <Root>/theta_y_dot_plot */
	this.urlHashMap["Sim_NL_XZ:7"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_XZ:7";
	/* <Root>/theta_y_plot */
	this.urlHashMap["Sim_NL_XZ:8"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_XZ:8";
	/* <S1>/Output */
	this.urlHashMap["Sim_NL_XZ:14:1"] = "Sim_NL_XZ.c:97&Sim_NL_XZ.h:38";
	/* <S1>/White Noise */
	this.urlHashMap["Sim_NL_XZ:14:2"] = "Sim_NL_XZ.c:77,98,159&Sim_NL_XZ.h:40,41";
	/* <S2>:1 */
	this.urlHashMap["Sim_NL_XZ:4:1"] = "Sim_NL_XZ.c:111";
	/* <S2>:1:4 */
	this.urlHashMap["Sim_NL_XZ:4:1:4"] = "Sim_NL_XZ.c:112";
	/* <S2>:1:5 */
	this.urlHashMap["Sim_NL_XZ:4:1:5"] = "Sim_NL_XZ.c:113";
	/* <S2>:1:6 */
	this.urlHashMap["Sim_NL_XZ:4:1:6"] = "Sim_NL_XZ.c:114";
	/* <S2>:1:7 */
	this.urlHashMap["Sim_NL_XZ:4:1:7"] = "Sim_NL_XZ.c:115";
	/* <S2>:1:9 */
	this.urlHashMap["Sim_NL_XZ:4:1:9"] = "Sim_NL_XZ.c:116";
	/* <S2>:1:11 */
	this.urlHashMap["Sim_NL_XZ:4:1:11"] = "Sim_NL_XZ.c:119";
	/* <S2>:1:14 */
	this.urlHashMap["Sim_NL_XZ:4:1:14"] = "Sim_NL_XZ.c:123";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "Sim_NL_XZ"};
	this.sidHashMap["Sim_NL_XZ"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "Sim_NL_XZ:14"};
	this.sidHashMap["Sim_NL_XZ:14"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<S2>"] = {sid: "Sim_NL_XZ:4"};
	this.sidHashMap["Sim_NL_XZ:4"] = {rtwname: "<S2>"};
	this.rtwnameHashMap["<Root>/T_y "] = {sid: "Sim_NL_XZ:1"};
	this.sidHashMap["Sim_NL_XZ:1"] = {rtwname: "<Root>/T_y "};
	this.rtwnameHashMap["<Root>/Band-Limited White Noise"] = {sid: "Sim_NL_XZ:14"};
	this.sidHashMap["Sim_NL_XZ:14"] = {rtwname: "<Root>/Band-Limited White Noise"};
	this.rtwnameHashMap["<Root>/Demux"] = {sid: "Sim_NL_XZ:2"};
	this.sidHashMap["Sim_NL_XZ:2"] = {rtwname: "<Root>/Demux"};
	this.rtwnameHashMap["<Root>/Demux1"] = {sid: "Sim_NL_XZ:15"};
	this.sidHashMap["Sim_NL_XZ:15"] = {rtwname: "<Root>/Demux1"};
	this.rtwnameHashMap["<Root>/Integrator"] = {sid: "Sim_NL_XZ:3"};
	this.sidHashMap["Sim_NL_XZ:3"] = {rtwname: "<Root>/Integrator"};
	this.rtwnameHashMap["<Root>/MATLAB Function"] = {sid: "Sim_NL_XZ:4"};
	this.sidHashMap["Sim_NL_XZ:4"] = {rtwname: "<Root>/MATLAB Function"};
	this.rtwnameHashMap["<Root>/Mux"] = {sid: "Sim_NL_XZ:16"};
	this.sidHashMap["Sim_NL_XZ:16"] = {rtwname: "<Root>/Mux"};
	this.rtwnameHashMap["<Root>/Sum"] = {sid: "Sim_NL_XZ:17"};
	this.sidHashMap["Sim_NL_XZ:17"] = {rtwname: "<Root>/Sum"};
	this.rtwnameHashMap["<Root>/phi_y_dot_plot"] = {sid: "Sim_NL_XZ:5"};
	this.sidHashMap["Sim_NL_XZ:5"] = {rtwname: "<Root>/phi_y_dot_plot"};
	this.rtwnameHashMap["<Root>/phi_y_plot"] = {sid: "Sim_NL_XZ:6"};
	this.sidHashMap["Sim_NL_XZ:6"] = {rtwname: "<Root>/phi_y_plot"};
	this.rtwnameHashMap["<Root>/theta_y_dot_plot"] = {sid: "Sim_NL_XZ:7"};
	this.sidHashMap["Sim_NL_XZ:7"] = {rtwname: "<Root>/theta_y_dot_plot"};
	this.rtwnameHashMap["<Root>/theta_y_plot"] = {sid: "Sim_NL_XZ:8"};
	this.sidHashMap["Sim_NL_XZ:8"] = {rtwname: "<Root>/theta_y_plot"};
	this.rtwnameHashMap["<Root>/phi_y"] = {sid: "Sim_NL_XZ:9"};
	this.sidHashMap["Sim_NL_XZ:9"] = {rtwname: "<Root>/phi_y"};
	this.rtwnameHashMap["<Root>/theta_y"] = {sid: "Sim_NL_XZ:10"};
	this.sidHashMap["Sim_NL_XZ:10"] = {rtwname: "<Root>/theta_y"};
	this.rtwnameHashMap["<Root>/phi_y_dot"] = {sid: "Sim_NL_XZ:11"};
	this.sidHashMap["Sim_NL_XZ:11"] = {rtwname: "<Root>/phi_y_dot"};
	this.rtwnameHashMap["<Root>/theta_y_dot"] = {sid: "Sim_NL_XZ:12"};
	this.sidHashMap["Sim_NL_XZ:12"] = {rtwname: "<Root>/theta_y_dot"};
	this.rtwnameHashMap["<Root>/x"] = {sid: "Sim_NL_XZ:13"};
	this.sidHashMap["Sim_NL_XZ:13"] = {rtwname: "<Root>/x"};
	this.rtwnameHashMap["<S1>/Output"] = {sid: "Sim_NL_XZ:14:1"};
	this.sidHashMap["Sim_NL_XZ:14:1"] = {rtwname: "<S1>/Output"};
	this.rtwnameHashMap["<S1>/White Noise"] = {sid: "Sim_NL_XZ:14:2"};
	this.sidHashMap["Sim_NL_XZ:14:2"] = {rtwname: "<S1>/White Noise"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "Sim_NL_XZ:14:3"};
	this.sidHashMap["Sim_NL_XZ:14:3"] = {rtwname: "<S1>/y"};
	this.rtwnameHashMap["<S2>:1"] = {sid: "Sim_NL_XZ:4:1"};
	this.sidHashMap["Sim_NL_XZ:4:1"] = {rtwname: "<S2>:1"};
	this.rtwnameHashMap["<S2>:1:4"] = {sid: "Sim_NL_XZ:4:1:4"};
	this.sidHashMap["Sim_NL_XZ:4:1:4"] = {rtwname: "<S2>:1:4"};
	this.rtwnameHashMap["<S2>:1:5"] = {sid: "Sim_NL_XZ:4:1:5"};
	this.sidHashMap["Sim_NL_XZ:4:1:5"] = {rtwname: "<S2>:1:5"};
	this.rtwnameHashMap["<S2>:1:6"] = {sid: "Sim_NL_XZ:4:1:6"};
	this.sidHashMap["Sim_NL_XZ:4:1:6"] = {rtwname: "<S2>:1:6"};
	this.rtwnameHashMap["<S2>:1:7"] = {sid: "Sim_NL_XZ:4:1:7"};
	this.sidHashMap["Sim_NL_XZ:4:1:7"] = {rtwname: "<S2>:1:7"};
	this.rtwnameHashMap["<S2>:1:9"] = {sid: "Sim_NL_XZ:4:1:9"};
	this.sidHashMap["Sim_NL_XZ:4:1:9"] = {rtwname: "<S2>:1:9"};
	this.rtwnameHashMap["<S2>:1:11"] = {sid: "Sim_NL_XZ:4:1:11"};
	this.sidHashMap["Sim_NL_XZ:4:1:11"] = {rtwname: "<S2>:1:11"};
	this.rtwnameHashMap["<S2>:1:14"] = {sid: "Sim_NL_XZ:4:1:14"};
	this.sidHashMap["Sim_NL_XZ:4:1:14"] = {rtwname: "<S2>:1:14"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();

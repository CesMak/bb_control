function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Integrator */
	this.urlHashMap["Sim_NL_YZ:27"] = "Sim_NL_YZ.c:81,103,167&Sim_NL_YZ.h:46,51,56";
	/* <Root>/MATLAB Function */
	this.urlHashMap["Sim_NL_YZ:26"] = "Sim_NL_YZ.c:109&Sim_NL_YZ.h:37";
	/* <Root>/Sum */
	this.urlHashMap["Sim_NL_YZ:57"] = "Sim_NL_YZ.c:138&Sim_NL_YZ.h:39";
	/* <Root>/phi_x_dot_plot */
	this.urlHashMap["Sim_NL_YZ:38"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_YZ:38";
	/* <Root>/phi_x_plot */
	this.urlHashMap["Sim_NL_YZ:36"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_YZ:36";
	/* <Root>/theta_x_dot_plot */
	this.urlHashMap["Sim_NL_YZ:39"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_YZ:39";
	/* <Root>/theta_x_plot */
	this.urlHashMap["Sim_NL_YZ:37"] = "msg=rtwMsg_reducedBlock&block=Sim_NL_YZ:37";
	/* <S1>/Output */
	this.urlHashMap["Sim_NL_YZ:58:1"] = "Sim_NL_YZ.c:97&Sim_NL_YZ.h:38";
	/* <S1>/White Noise */
	this.urlHashMap["Sim_NL_YZ:58:2"] = "Sim_NL_YZ.c:77,98,159&Sim_NL_YZ.h:40,41";
	/* <S2>:1 */
	this.urlHashMap["Sim_NL_YZ:26:1"] = "Sim_NL_YZ.c:111";
	/* <S2>:1:4 */
	this.urlHashMap["Sim_NL_YZ:26:1:4"] = "Sim_NL_YZ.c:112";
	/* <S2>:1:5 */
	this.urlHashMap["Sim_NL_YZ:26:1:5"] = "Sim_NL_YZ.c:113";
	/* <S2>:1:6 */
	this.urlHashMap["Sim_NL_YZ:26:1:6"] = "Sim_NL_YZ.c:114";
	/* <S2>:1:7 */
	this.urlHashMap["Sim_NL_YZ:26:1:7"] = "Sim_NL_YZ.c:115";
	/* <S2>:1:9 */
	this.urlHashMap["Sim_NL_YZ:26:1:9"] = "Sim_NL_YZ.c:116";
	/* <S2>:1:11 */
	this.urlHashMap["Sim_NL_YZ:26:1:11"] = "Sim_NL_YZ.c:119";
	/* <S2>:1:14 */
	this.urlHashMap["Sim_NL_YZ:26:1:14"] = "Sim_NL_YZ.c:123";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "Sim_NL_YZ"};
	this.sidHashMap["Sim_NL_YZ"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "Sim_NL_YZ:58"};
	this.sidHashMap["Sim_NL_YZ:58"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<S2>"] = {sid: "Sim_NL_YZ:26"};
	this.sidHashMap["Sim_NL_YZ:26"] = {rtwname: "<S2>"};
	this.rtwnameHashMap["<Root>/T_x "] = {sid: "Sim_NL_YZ:41"};
	this.sidHashMap["Sim_NL_YZ:41"] = {rtwname: "<Root>/T_x "};
	this.rtwnameHashMap["<Root>/Band-Limited White Noise"] = {sid: "Sim_NL_YZ:58"};
	this.sidHashMap["Sim_NL_YZ:58"] = {rtwname: "<Root>/Band-Limited White Noise"};
	this.rtwnameHashMap["<Root>/Demux"] = {sid: "Sim_NL_YZ:28"};
	this.sidHashMap["Sim_NL_YZ:28"] = {rtwname: "<Root>/Demux"};
	this.rtwnameHashMap["<Root>/Demux1"] = {sid: "Sim_NL_YZ:55"};
	this.sidHashMap["Sim_NL_YZ:55"] = {rtwname: "<Root>/Demux1"};
	this.rtwnameHashMap["<Root>/Integrator"] = {sid: "Sim_NL_YZ:27"};
	this.sidHashMap["Sim_NL_YZ:27"] = {rtwname: "<Root>/Integrator"};
	this.rtwnameHashMap["<Root>/MATLAB Function"] = {sid: "Sim_NL_YZ:26"};
	this.sidHashMap["Sim_NL_YZ:26"] = {rtwname: "<Root>/MATLAB Function"};
	this.rtwnameHashMap["<Root>/Mux"] = {sid: "Sim_NL_YZ:56"};
	this.sidHashMap["Sim_NL_YZ:56"] = {rtwname: "<Root>/Mux"};
	this.rtwnameHashMap["<Root>/Sum"] = {sid: "Sim_NL_YZ:57"};
	this.sidHashMap["Sim_NL_YZ:57"] = {rtwname: "<Root>/Sum"};
	this.rtwnameHashMap["<Root>/phi_x_dot_plot"] = {sid: "Sim_NL_YZ:38"};
	this.sidHashMap["Sim_NL_YZ:38"] = {rtwname: "<Root>/phi_x_dot_plot"};
	this.rtwnameHashMap["<Root>/phi_x_plot"] = {sid: "Sim_NL_YZ:36"};
	this.sidHashMap["Sim_NL_YZ:36"] = {rtwname: "<Root>/phi_x_plot"};
	this.rtwnameHashMap["<Root>/theta_x_dot_plot"] = {sid: "Sim_NL_YZ:39"};
	this.sidHashMap["Sim_NL_YZ:39"] = {rtwname: "<Root>/theta_x_dot_plot"};
	this.rtwnameHashMap["<Root>/theta_x_plot"] = {sid: "Sim_NL_YZ:37"};
	this.sidHashMap["Sim_NL_YZ:37"] = {rtwname: "<Root>/theta_x_plot"};
	this.rtwnameHashMap["<Root>/phi_x"] = {sid: "Sim_NL_YZ:40"};
	this.sidHashMap["Sim_NL_YZ:40"] = {rtwname: "<Root>/phi_x"};
	this.rtwnameHashMap["<Root>/theta_x"] = {sid: "Sim_NL_YZ:42"};
	this.sidHashMap["Sim_NL_YZ:42"] = {rtwname: "<Root>/theta_x"};
	this.rtwnameHashMap["<Root>/phi_x_dot"] = {sid: "Sim_NL_YZ:43"};
	this.sidHashMap["Sim_NL_YZ:43"] = {rtwname: "<Root>/phi_x_dot"};
	this.rtwnameHashMap["<Root>/theta_x_dot"] = {sid: "Sim_NL_YZ:44"};
	this.sidHashMap["Sim_NL_YZ:44"] = {rtwname: "<Root>/theta_x_dot"};
	this.rtwnameHashMap["<Root>/x"] = {sid: "Sim_NL_YZ:45"};
	this.sidHashMap["Sim_NL_YZ:45"] = {rtwname: "<Root>/x"};
	this.rtwnameHashMap["<S1>/Output"] = {sid: "Sim_NL_YZ:58:1"};
	this.sidHashMap["Sim_NL_YZ:58:1"] = {rtwname: "<S1>/Output"};
	this.rtwnameHashMap["<S1>/White Noise"] = {sid: "Sim_NL_YZ:58:2"};
	this.sidHashMap["Sim_NL_YZ:58:2"] = {rtwname: "<S1>/White Noise"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "Sim_NL_YZ:58:3"};
	this.sidHashMap["Sim_NL_YZ:58:3"] = {rtwname: "<S1>/y"};
	this.rtwnameHashMap["<S2>:1"] = {sid: "Sim_NL_YZ:26:1"};
	this.sidHashMap["Sim_NL_YZ:26:1"] = {rtwname: "<S2>:1"};
	this.rtwnameHashMap["<S2>:1:4"] = {sid: "Sim_NL_YZ:26:1:4"};
	this.sidHashMap["Sim_NL_YZ:26:1:4"] = {rtwname: "<S2>:1:4"};
	this.rtwnameHashMap["<S2>:1:5"] = {sid: "Sim_NL_YZ:26:1:5"};
	this.sidHashMap["Sim_NL_YZ:26:1:5"] = {rtwname: "<S2>:1:5"};
	this.rtwnameHashMap["<S2>:1:6"] = {sid: "Sim_NL_YZ:26:1:6"};
	this.sidHashMap["Sim_NL_YZ:26:1:6"] = {rtwname: "<S2>:1:6"};
	this.rtwnameHashMap["<S2>:1:7"] = {sid: "Sim_NL_YZ:26:1:7"};
	this.sidHashMap["Sim_NL_YZ:26:1:7"] = {rtwname: "<S2>:1:7"};
	this.rtwnameHashMap["<S2>:1:9"] = {sid: "Sim_NL_YZ:26:1:9"};
	this.sidHashMap["Sim_NL_YZ:26:1:9"] = {rtwname: "<S2>:1:9"};
	this.rtwnameHashMap["<S2>:1:11"] = {sid: "Sim_NL_YZ:26:1:11"};
	this.sidHashMap["Sim_NL_YZ:26:1:11"] = {rtwname: "<S2>:1:11"};
	this.rtwnameHashMap["<S2>:1:14"] = {sid: "Sim_NL_YZ:26:1:14"};
	this.sidHashMap["Sim_NL_YZ:26:1:14"] = {rtwname: "<S2>:1:14"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();

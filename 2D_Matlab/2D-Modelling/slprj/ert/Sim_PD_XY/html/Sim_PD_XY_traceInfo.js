function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Gain */
	this.urlHashMap["Sim_PD_XY:39"] = "Sim_PD_XY.c:114&Sim_PD_XY.h:38";
	/* <Root>/Integrator */
	this.urlHashMap["Sim_PD_XY:37"] = "Sim_PD_XY.c:87,118,145&Sim_PD_XY.h:48,55,62";
	/* <Root>/Integrator1 */
	this.urlHashMap["Sim_PD_XY:38"] = "Sim_PD_XY.c:81,103,137&Sim_PD_XY.h:46,53,60";
	/* <Root>/Sum */
	this.urlHashMap["Sim_PD_XY:28"] = "Sim_PD_XY.c:106&Sim_PD_XY.h:37";
	/* <Root>/Sum1 */
	this.urlHashMap["Sim_PD_XY:43"] = "Sim_PD_XY.c:117&Sim_PD_XY.h:39";
	/* <Root>/Transfer Fcn1 */
	this.urlHashMap["Sim_PD_XY:34"] = "Sim_PD_XY.c:84,109,140&Sim_PD_XY.h:47,54,61";
	/* <Root>/theta_z_plot */
	this.urlHashMap["Sim_PD_XY:29"] = "msg=rtwMsg_SimulationReducedBlock&block=Sim_PD_XY:29";
	/* <S1>/Output */
	this.urlHashMap["Sim_PD_XY:44:1"] = "Sim_PD_XY.c:97&Sim_PD_XY.h:36";
	/* <S1>/White Noise */
	this.urlHashMap["Sim_PD_XY:44:2"] = "Sim_PD_XY.c:77,98,128&Sim_PD_XY.h:40,41";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "Sim_PD_XY"};
	this.sidHashMap["Sim_PD_XY"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "Sim_PD_XY:44"};
	this.sidHashMap["Sim_PD_XY:44"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<Root>/theta_z_AP "] = {sid: "Sim_PD_XY:40"};
	this.sidHashMap["Sim_PD_XY:40"] = {rtwname: "<Root>/theta_z_AP "};
	this.rtwnameHashMap["<Root>/Band-Limited White Noise"] = {sid: "Sim_PD_XY:44"};
	this.sidHashMap["Sim_PD_XY:44"] = {rtwname: "<Root>/Band-Limited White Noise"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "Sim_PD_XY:39"};
	this.sidHashMap["Sim_PD_XY:39"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/Integrator"] = {sid: "Sim_PD_XY:37"};
	this.sidHashMap["Sim_PD_XY:37"] = {rtwname: "<Root>/Integrator"};
	this.rtwnameHashMap["<Root>/Integrator1"] = {sid: "Sim_PD_XY:38"};
	this.sidHashMap["Sim_PD_XY:38"] = {rtwname: "<Root>/Integrator1"};
	this.rtwnameHashMap["<Root>/Sum"] = {sid: "Sim_PD_XY:28"};
	this.sidHashMap["Sim_PD_XY:28"] = {rtwname: "<Root>/Sum"};
	this.rtwnameHashMap["<Root>/Sum1"] = {sid: "Sim_PD_XY:43"};
	this.sidHashMap["Sim_PD_XY:43"] = {rtwname: "<Root>/Sum1"};
	this.rtwnameHashMap["<Root>/Transfer Fcn1"] = {sid: "Sim_PD_XY:34"};
	this.sidHashMap["Sim_PD_XY:34"] = {rtwname: "<Root>/Transfer Fcn1"};
	this.rtwnameHashMap["<Root>/theta_z_plot"] = {sid: "Sim_PD_XY:29"};
	this.sidHashMap["Sim_PD_XY:29"] = {rtwname: "<Root>/theta_z_plot"};
	this.rtwnameHashMap["<Root>/theta_z "] = {sid: "Sim_PD_XY:41"};
	this.sidHashMap["Sim_PD_XY:41"] = {rtwname: "<Root>/theta_z "};
	this.rtwnameHashMap["<Root>/T_z"] = {sid: "Sim_PD_XY:42"};
	this.sidHashMap["Sim_PD_XY:42"] = {rtwname: "<Root>/T_z"};
	this.rtwnameHashMap["<S1>/Output"] = {sid: "Sim_PD_XY:44:1"};
	this.sidHashMap["Sim_PD_XY:44:1"] = {rtwname: "<S1>/Output"};
	this.rtwnameHashMap["<S1>/White Noise"] = {sid: "Sim_PD_XY:44:2"};
	this.sidHashMap["Sim_PD_XY:44:2"] = {rtwname: "<S1>/White Noise"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "Sim_PD_XY:44:3"};
	this.sidHashMap["Sim_PD_XY:44:3"] = {rtwname: "<S1>/y"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();

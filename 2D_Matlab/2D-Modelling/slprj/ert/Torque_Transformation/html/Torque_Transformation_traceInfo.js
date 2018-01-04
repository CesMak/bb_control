function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Gain */
	this.urlHashMap["Torque_Transformation:37"] = "Torque_Transformation.c:57";
	/* <Root>/Gain1 */
	this.urlHashMap["Torque_Transformation:39"] = "Torque_Transformation.c:58";
	/* <Root>/Gain10 */
	this.urlHashMap["Torque_Transformation:49"] = "Torque_Transformation.c:67";
	/* <Root>/Gain11 */
	this.urlHashMap["Torque_Transformation:50"] = "Torque_Transformation.c:68";
	/* <Root>/Gain12 */
	this.urlHashMap["Torque_Transformation:55"] = "Torque_Transformation.c:41";
	/* <Root>/Gain13 */
	this.urlHashMap["Torque_Transformation:56"] = "Torque_Transformation.c:42";
	/* <Root>/Gain14 */
	this.urlHashMap["Torque_Transformation:57"] = "Torque_Transformation.c:43";
	/* <Root>/Gain15 */
	this.urlHashMap["Torque_Transformation:58"] = "Torque_Transformation.c:44";
	/* <Root>/Gain16 */
	this.urlHashMap["Torque_Transformation:60"] = "Torque_Transformation.c:45";
	/* <Root>/Gain2 */
	this.urlHashMap["Torque_Transformation:40"] = "Torque_Transformation.c:69";
	/* <Root>/Gain4 */
	this.urlHashMap["Torque_Transformation:42"] = "Torque_Transformation.c:59";
	/* <Root>/Gain5 */
	this.urlHashMap["Torque_Transformation:43"] = "Torque_Transformation.c:56";
	/* <Root>/Gain6 */
	this.urlHashMap["Torque_Transformation:44"] = "Torque_Transformation.c:70";
	/* <Root>/Gain7 */
	this.urlHashMap["Torque_Transformation:45"] = "Torque_Transformation.c:66";
	/* <Root>/Gain8 */
	this.urlHashMap["Torque_Transformation:46"] = "Torque_Transformation.c:71";
	/* <Root>/Gain9 */
	this.urlHashMap["Torque_Transformation:48"] = "Torque_Transformation.c:46";
	/* <Root>/Sum */
	this.urlHashMap["Torque_Transformation:32"] = "Torque_Transformation.c:60";
	/* <Root>/Sum1 */
	this.urlHashMap["Torque_Transformation:33"] = "Torque_Transformation.c:61";
	/* <Root>/Sum2 */
	this.urlHashMap["Torque_Transformation:34"] = "Torque_Transformation.c:72";
	/* <Root>/Sum3 */
	this.urlHashMap["Torque_Transformation:35"] = "Torque_Transformation.c:73";
	/* <Root>/Sum4 */
	this.urlHashMap["Torque_Transformation:36"] = "Torque_Transformation.c:74";
	/* <Root>/Sum5 */
	this.urlHashMap["Torque_Transformation:51"] = "Torque_Transformation.c:75";
	/* <Root>/Sum6 */
	this.urlHashMap["Torque_Transformation:52"] = "Torque_Transformation.c:47";
	/* <Root>/Sum7 */
	this.urlHashMap["Torque_Transformation:53"] = "Torque_Transformation.c:48";
	/* <Root>/Sum8 */
	this.urlHashMap["Torque_Transformation:54"] = "Torque_Transformation.c:49";
	/* <Root>/Sum9 */
	this.urlHashMap["Torque_Transformation:59"] = "Torque_Transformation.c:50";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "Torque_Transformation"};
	this.sidHashMap["Torque_Transformation"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<Root>/T_x"] = {sid: "Torque_Transformation:25"};
	this.sidHashMap["Torque_Transformation:25"] = {rtwname: "<Root>/T_x"};
	this.rtwnameHashMap["<Root>/T_y"] = {sid: "Torque_Transformation:26"};
	this.sidHashMap["Torque_Transformation:26"] = {rtwname: "<Root>/T_y"};
	this.rtwnameHashMap["<Root>/T_z"] = {sid: "Torque_Transformation:27"};
	this.sidHashMap["Torque_Transformation:27"] = {rtwname: "<Root>/T_z"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "Torque_Transformation:37"};
	this.sidHashMap["Torque_Transformation:37"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/Gain1"] = {sid: "Torque_Transformation:39"};
	this.sidHashMap["Torque_Transformation:39"] = {rtwname: "<Root>/Gain1"};
	this.rtwnameHashMap["<Root>/Gain10"] = {sid: "Torque_Transformation:49"};
	this.sidHashMap["Torque_Transformation:49"] = {rtwname: "<Root>/Gain10"};
	this.rtwnameHashMap["<Root>/Gain11"] = {sid: "Torque_Transformation:50"};
	this.sidHashMap["Torque_Transformation:50"] = {rtwname: "<Root>/Gain11"};
	this.rtwnameHashMap["<Root>/Gain12"] = {sid: "Torque_Transformation:55"};
	this.sidHashMap["Torque_Transformation:55"] = {rtwname: "<Root>/Gain12"};
	this.rtwnameHashMap["<Root>/Gain13"] = {sid: "Torque_Transformation:56"};
	this.sidHashMap["Torque_Transformation:56"] = {rtwname: "<Root>/Gain13"};
	this.rtwnameHashMap["<Root>/Gain14"] = {sid: "Torque_Transformation:57"};
	this.sidHashMap["Torque_Transformation:57"] = {rtwname: "<Root>/Gain14"};
	this.rtwnameHashMap["<Root>/Gain15"] = {sid: "Torque_Transformation:58"};
	this.sidHashMap["Torque_Transformation:58"] = {rtwname: "<Root>/Gain15"};
	this.rtwnameHashMap["<Root>/Gain16"] = {sid: "Torque_Transformation:60"};
	this.sidHashMap["Torque_Transformation:60"] = {rtwname: "<Root>/Gain16"};
	this.rtwnameHashMap["<Root>/Gain2"] = {sid: "Torque_Transformation:40"};
	this.sidHashMap["Torque_Transformation:40"] = {rtwname: "<Root>/Gain2"};
	this.rtwnameHashMap["<Root>/Gain4"] = {sid: "Torque_Transformation:42"};
	this.sidHashMap["Torque_Transformation:42"] = {rtwname: "<Root>/Gain4"};
	this.rtwnameHashMap["<Root>/Gain5"] = {sid: "Torque_Transformation:43"};
	this.sidHashMap["Torque_Transformation:43"] = {rtwname: "<Root>/Gain5"};
	this.rtwnameHashMap["<Root>/Gain6"] = {sid: "Torque_Transformation:44"};
	this.sidHashMap["Torque_Transformation:44"] = {rtwname: "<Root>/Gain6"};
	this.rtwnameHashMap["<Root>/Gain7"] = {sid: "Torque_Transformation:45"};
	this.sidHashMap["Torque_Transformation:45"] = {rtwname: "<Root>/Gain7"};
	this.rtwnameHashMap["<Root>/Gain8"] = {sid: "Torque_Transformation:46"};
	this.sidHashMap["Torque_Transformation:46"] = {rtwname: "<Root>/Gain8"};
	this.rtwnameHashMap["<Root>/Gain9"] = {sid: "Torque_Transformation:48"};
	this.sidHashMap["Torque_Transformation:48"] = {rtwname: "<Root>/Gain9"};
	this.rtwnameHashMap["<Root>/Sum"] = {sid: "Torque_Transformation:32"};
	this.sidHashMap["Torque_Transformation:32"] = {rtwname: "<Root>/Sum"};
	this.rtwnameHashMap["<Root>/Sum1"] = {sid: "Torque_Transformation:33"};
	this.sidHashMap["Torque_Transformation:33"] = {rtwname: "<Root>/Sum1"};
	this.rtwnameHashMap["<Root>/Sum2"] = {sid: "Torque_Transformation:34"};
	this.sidHashMap["Torque_Transformation:34"] = {rtwname: "<Root>/Sum2"};
	this.rtwnameHashMap["<Root>/Sum3"] = {sid: "Torque_Transformation:35"};
	this.sidHashMap["Torque_Transformation:35"] = {rtwname: "<Root>/Sum3"};
	this.rtwnameHashMap["<Root>/Sum4"] = {sid: "Torque_Transformation:36"};
	this.sidHashMap["Torque_Transformation:36"] = {rtwname: "<Root>/Sum4"};
	this.rtwnameHashMap["<Root>/Sum5"] = {sid: "Torque_Transformation:51"};
	this.sidHashMap["Torque_Transformation:51"] = {rtwname: "<Root>/Sum5"};
	this.rtwnameHashMap["<Root>/Sum6"] = {sid: "Torque_Transformation:52"};
	this.sidHashMap["Torque_Transformation:52"] = {rtwname: "<Root>/Sum6"};
	this.rtwnameHashMap["<Root>/Sum7"] = {sid: "Torque_Transformation:53"};
	this.sidHashMap["Torque_Transformation:53"] = {rtwname: "<Root>/Sum7"};
	this.rtwnameHashMap["<Root>/Sum8"] = {sid: "Torque_Transformation:54"};
	this.sidHashMap["Torque_Transformation:54"] = {rtwname: "<Root>/Sum8"};
	this.rtwnameHashMap["<Root>/Sum9"] = {sid: "Torque_Transformation:59"};
	this.sidHashMap["Torque_Transformation:59"] = {rtwname: "<Root>/Sum9"};
	this.rtwnameHashMap["<Root>/T1"] = {sid: "Torque_Transformation:1"};
	this.sidHashMap["Torque_Transformation:1"] = {rtwname: "<Root>/T1"};
	this.rtwnameHashMap["<Root>/T2"] = {sid: "Torque_Transformation:22"};
	this.sidHashMap["Torque_Transformation:22"] = {rtwname: "<Root>/T2"};
	this.rtwnameHashMap["<Root>/T3"] = {sid: "Torque_Transformation:23"};
	this.sidHashMap["Torque_Transformation:23"] = {rtwname: "<Root>/T3"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();

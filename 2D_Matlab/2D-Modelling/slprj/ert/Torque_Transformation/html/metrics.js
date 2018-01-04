function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.fcn["Torque_Transformatio_initialize"] = {file: "D:\\BallBot\\Controller\\bb_control\\slprj\\ert\\Torque_Transformation\\Torque_Transformation.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["Torque_Transformation"] = {file: "D:\\BallBot\\Controller\\bb_control\\slprj\\ert\\Torque_Transformation\\Torque_Transformation.c",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data;}
}
	 CodeMetrics.instance = new CodeMetrics();

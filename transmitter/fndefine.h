
extern int GetDegree(double); // Get Degree in Degree Distribution Table by ProbValue.
extern int GetLayer(double, double *);
extern void splitter(void);
extern double *RaptorDistribution(int);
extern double *LayerProGenerator(void);
extern void assembler(void);
extern void encoder(void);
extern double *LayerProGenerator(double *);
extern void splitterMB(void);
extern void WBcalculator(uint64_t block_index);
extern void receiverdown(void);
extern void Recorder(void);
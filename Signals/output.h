#ifndef _OUTPUT_H_
#define _OUTPUT_H_
#include "Signals.h"
 class output
{
public:
    output();
    ~output();
	Signals *_signals;
	inputs *_inputs;
    void DoPathLoss(char *filename, unsigned char geo, unsigned char kml,
            unsigned char ngs, struct site *xmtr, unsigned char txsites);
    void DoSigStr(char *filename, unsigned char geo, unsigned char kml,
              unsigned char ngs, struct site *xmtr, unsigned char txsites);
    void DoRxdPwr(char *filename, unsigned char geo, unsigned char kml,
              unsigned char ngs, struct site *xmtr, unsigned char txsites);
    void DoLOS(char *filename, unsigned char geo, unsigned char kml,
           unsigned char ngs, struct site *xmtr, unsigned char txsites);
    void PathReport(struct site source, struct site destination, char *name,
            char graph_it, int propmodel, int pmenv, double rxGain);
    void SeriesData(struct site source, struct site destination, char *name,
            unsigned char fresnel_plot, unsigned char normalised);
};


#endif /* _OUTPUT_H_ */

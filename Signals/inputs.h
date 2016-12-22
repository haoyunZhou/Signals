#ifndef _INPUTS_H_
#define _INPUTS_H_

#include "common.h"
class inputs
{
public:
    inputs();
    ~inputs();
    int LoadSDF_SDF(char *name, int winfiles);
    char LoadSDF(char *name, int winfiles);
    void LoadPAT(char *filename);
    void LoadSignalColors(struct site xmtr);
    void LoadLossColors(struct site xmtr);
    void LoadDBMColors(struct site xmtr);
    void LoadTopoData(int max_lon, int min_lon, int max_lat, int min_lat);
    void LoadUDT(char *filename);
    int loadLIDAR(char *filename);
    int loadClutter(char *filename, double radius, struct site tx);
protected:
private:
}


#endif /* _INPUTS_H_ */

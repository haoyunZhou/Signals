#include <iostream>
#include "common.h"
#include "main.h"
using namespace std;

main::main()
{
}

main::~main()
{
}
int main::doMain(int argc, char *argv[])
{
    int x, y, z = 0, min_lat, min_lon, max_lat, max_lon,
        rxlat, rxlon, txlat, txlon, west_min, west_max,
        nortRxHin, nortRxHax, propmodel, knifeedge = 0, ppa =
        0, normalise = 0, haf = 0, pmenv = 1, lidar=0;

    bool use_threads = true;

    unsigned char LRmap = 0, txsites = 0, topomap = 0, geo = 0, kml =
        0, area_mode = 0, max_txsites, ngs = 0;

    char mapfile[255], udt_file[255], ano_filename[255], lidar_tiles[4096], clutter_file[255];

    double altitude = 0.0, altitudeLR = 0.0, tx_range = 0.0,
        rx_range = 0.0, deg_range = 0.0, deg_limit = 0.0, deg_range_lon;

    if (strstr(argv[0], "signalserverHD")) {
        MAXPAGES = 9;
        ARRAYSIZE = 32410;
        IPPD = 3600;
    }

    if (strstr(argv[0], "signalserverLIDAR")) {
        MAXPAGES = 100; // 10x10
        lidar = 1;
        IPPD = 5000; // will be overridden based upon file header...
    }

    strncpy(ss_name, "Signal Server\0", 14);

    if (argc == 1) {

        fprintf(stdout, "Version: %s %.2f (Built for %d DEM tiles at %d pixels)\n", ss_name, version,MAXPAGES, IPPD);
        fprintf(stdout, "License: GNU General Public License (GPL) version 2\n\n");
        fprintf(stdout, "Radio propagation simulator by Alex Farrant QCVS, 2E0TDW\n");
        fprintf(stdout, "Based upon SPLAT! by John Magliacane, KD2BD\n\n");
        fprintf(stdout, "Usage: signalserver [data options] [input options] [output options] -o outputfile\n\n");
        fprintf(stdout, "Data:\n");
        fprintf(stdout, "     -sdf Directory containing SRTM derived .sdf DEM tiles\n");
        fprintf(stdout, "     -lid ASCII grid tile (LIDAR) with dimensions and resolution defined in header\n");
        fprintf(stdout, "     -udt User defined point clutter as decimal co-ordinates: 'latitude,longitude,height'\n");
        fprintf(stdout, "     -clt MODIS 17-class wide area clutter in ASCII grid format\n");
        fprintf(stdout, "Input:\n");
        fprintf(stdout,	"     -lat Tx Latitude (decimal degrees) -70/+70\n");
        fprintf(stdout,	"     -lon Tx Longitude (decimal degrees) -180/+180\n");
        fprintf(stdout, "     -txh Tx Height (above ground)\n");
        fprintf(stdout,	"     -rla (Optional) Rx Latitude for PPA (decimal degrees) -70/+70\n");
        fprintf(stdout, "     -rlo (Optional) Rx Longitude for PPA (decimal degrees) -180/+180\n");
        fprintf(stdout,	"     -f Tx Frequency (MHz) 20MHz to 100GHz (LOS after 20GHz)\n");
        fprintf(stdout,	"     -erp Tx Effective Radiated Power (Watts) including Tx+Rx gain\n");
        fprintf(stdout,	"     -rxh Rx Height(s) (optional. Default=0.1)\n");
        fprintf(stdout,	"     -rxg Rx gain dBi (optional for text report)\n");
        fprintf(stdout,	"     -hp Horizontal Polarisation (default=vertical)\n");
        fprintf(stdout, "     -gc Random ground clutter (feet/meters)\n");
        fprintf(stdout, "     -m Metric units of measurement\n");
        fprintf(stdout, "     -te Terrain code 1-6 (optional)\n");
        fprintf(stdout,	"     -terdic Terrain dielectric value 2-80 (optional)\n");
        fprintf(stdout,	"     -tercon Terrain conductivity 0.01-0.0001 (optional)\n");
        fprintf(stdout, "     -cl Climate code 1-6 (optional)\n");
        fprintf(stdout, "Output:\n");
        fprintf(stdout,	"     -dbm Plot Rxd signal power instead of field strength\n");
        fprintf(stdout, "     -rt Rx Threshold (dB / dBm / dBuV/m)\n");
        fprintf(stdout, "     -o Filename. Required. \n");
        fprintf(stdout, "     -R Radius (miles/kilometers)\n");
        fprintf(stdout,	"     -res Pixels per tile. 300/600/1200/3600 (Optional. LIDAR res is within the tile)\n");
        fprintf(stdout,	"     -pm Propagation model. 1: ITM, 2: LOS, 3: Hata, 4: ECC33,\n");
        fprintf(stdout,	"     	  5: SUI, 6: COST-Hata, 7: FSPL, 8: ITWOM, 9: Ericsson, 10: Plane earth\n");
        fprintf(stdout,	"     -pe Propagation model mode: 1=Urban,2=Suburban,3=Rural\n");
        fprintf(stdout,	"     -ked Knife edge diffraction (Already on for ITM)\n");
        fprintf(stdout, "Debugging:\n");
        fprintf(stdout, "     -t Terrain greyscale background\n");
        fprintf(stdout, "     -dbg Verbose debug messages\n");
        fprintf(stdout, "     -ng Normalise Path Profile graph\n");
        fprintf(stdout, "     -haf Halve 1 or 2 (optional)\n");
        fprintf(stdout, "     -nothreads Turn off threaded processing\n");

        fflush(stdout);

        return 1;
    }

    /*
     * If we're not called as signalserverLIDAR we can allocate various
     * memory now. For LIDAR we need to wait until we've parsed
     * the headers in the .asc file to know how much memory to allocate...
     */
    if (!lidar)
        do_allocs();

    y = argc - 1;
    kml = 0;
    geo = 0;
    dbm = 0;
    gpsav = 0;
    metric = 0;
    string[0] = 0;
    mapfile[0] = 0;
    clutter_file[0] = 0;
    clutter = 0.0;
    forced_erp = -1.0;
    forced_freq = 0.0;
    sdf_path[0] = 0;
    udt_file[0] = 0;
    path.length = 0;
    max_txsites = 30;
    fzone_clearance = 0.6;
    contour_threshold = 0;

    ano_filename[0] = 0;
    earthradius = EARTHRADIUS;
    max_range = 1.0;
    propmodel = 1;		//ITM
    lat = 0;
    lon = 0;
    txh = 0;
    ngs = 1;		// no terrain background
    kml = 1;
    LRmap = 1;
    area_mode = 1;
    ippd = IPPD;		// default resolution

    sscanf("0.1", "%lf", &altitudeLR);

    // Defaults
    LR.eps_dielect = 15.0;	// Farmland
    LR.sgm_conductivity = 0.005;	// Farmland
    LR.eno_ns_surfref = 301.0;
    LR.frq_mhz = 19.0;	// Deliberately too low
    LR.radio_climate = 5;	// continental
    LR.pol = 1;		// vert
    LR.conf = 0.50;
    LR.rel = 0.50;
    LR.erp = 0.0;		// will default to Path Loss

    tx_site[0].lat = 91.0;
    tx_site[0].lon = 361.0;
    tx_site[1].lat = 91.0;
    tx_site[1].lon = 361.0;

    /* Scan for command line arguments */

    for (x = 1; x <= y; x++) {


        if (strcmp(argv[x], "-R") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &max_range);

            }
        }

        if (strcmp(argv[x], "-gc") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &clutter);

                if (clutter < 0.0)
                    clutter = 0.0;
            }
        }

        if (strcmp(argv[x], "-clt") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                strncpy(clutter_file, argv[z], 253);
            }
        }

        if (strcmp(argv[x], "-o") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                strncpy(mapfile, argv[z], 253);
                strncpy(tx_site[0].name, "Tx", 2);
                strncpy(tx_site[0].filename, argv[z], 253);
                LoadPAT(argv[z]);

            }
        }

        if (strcmp(argv[x], "-rt") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0])	/* A minus argument is legal here */
                sscanf(argv[z], "%d", &contour_threshold);
        }

        if (strcmp(argv[x], "-m") == 0) {
            metric = 1;

        }

        if (strcmp(argv[x], "-t") == 0) {
            ngs = 0;	// greyscale background
        }

        if (strcmp(argv[x], "-dbm") == 0)
            dbm = 1;

        if (strcmp(argv[x], "-sdf") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-')
                strncpy(sdf_path, argv[z], 253);
        }

        if (strcmp(argv[x], "-lid") == 0) {
            z = x + 1;
            lidar=1;
            if (z <= y && argv[z][0] && argv[z][0] != '-')
                strncpy(lidar_tiles, argv[z], 4094);
        }

        if (strcmp(argv[x], "-res") == 0) {
            z = x + 1;

            if (!lidar &&
                z <= y &&
                argv[z][0] &&
                argv[z][0] != '-') {
                sscanf(argv[z], "%d", &ippd);

                switch (ippd) {
                case 300:
                    MAXRAD = 500;
                    jgets = 3; // 3 dummy reads
                    break;
                case 600:
                    MAXRAD = 500;
                    jgets = 1;
                    break;
                case 1200:
                    MAXRAD = 200;
                    ippd = 1200;
                    break;
                case 3600:
                    MAXRAD = 100;
                    ippd = 3600;
                    break;
                default:
                    MAXRAD = 200;
                    ippd = 1200;
                    break;
                }
            }
        }

        if (strcmp(argv[x], "-lat") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0]) {
                tx_site[0].lat = ReadBearing(argv[z]);
            }
        }
        if (strcmp(argv[x], "-lon") == 0) {
            z = x + 1;
            if (z <= y && argv[z][0]) {
                tx_site[0].lon = ReadBearing(argv[z]);
                tx_site[0].lon *= -1;
                if (tx_site[0].lon < 0.0)
                    tx_site[0].lon += 360.0;
            }
        }
        //Switch to Path Profile Mode if Rx co-ords specified
        if (strcmp(argv[x], "-rla") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0]) {
                ppa = 1;
                tx_site[1].lat = ReadBearing(argv[z]);

            }
        }
        if (strcmp(argv[x], "-rlo") == 0) {
            z = x + 1;
            if (z <= y && argv[z][0]) {
                tx_site[1].lon = ReadBearing(argv[z]);
                tx_site[1].lon *= -1;
                if (tx_site[1].lon < 0.0)
                    tx_site[1].lon += 360.0;
            }
        }

        if (strcmp(argv[x], "-txh") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%f", &tx_site[0].alt);

            }
            txsites = 1;
        }

        if (strcmp(argv[x], "-rxh") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &altitudeLR);
                sscanf(argv[z], "%f", &tx_site[1].alt);
            }
        }

        if (strcmp(argv[x], "-rxg") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &rxGain);
            }
        }

        if (strcmp(argv[x], "-f") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &LR.frq_mhz);
            }
        }

        if (strcmp(argv[x], "-erp") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {
                sscanf(argv[z], "%lf", &LR.erp);
            }
        }

        if (strcmp(argv[x], "-cl") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {

                sscanf(argv[z], "%d", &LR.radio_climate);

            }
        }
        if (strcmp(argv[x], "-te") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {

                sscanf(argv[z], "%d", &ter);

                switch (ter) {
                case 1:	// Water
                    terdic = 80;
                    tercon = 0.010;
                    break;

                case 2:	// Marsh
                    terdic = 12;
                    tercon = 0.007;
                    break;

                case 3:	// Farmland
                    terdic = 15;
                    tercon = 0.005;
                    break;

                case 4:	//Mountain
                    terdic = 13;
                    tercon = 0.002;
                    break;
                case 5:	//Desert
                    terdic = 13;
                    tercon = 0.002;
                    break;
                case 6:	//Urban
                    terdic = 5;
                    tercon = 0.001;
                    break;
                }
                LR.eps_dielect = terdic;
                LR.sgm_conductivity = tercon;

            }
        }

        if (strcmp(argv[x], "-terdic") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {

                sscanf(argv[z], "%lf", &terdic);

                LR.eps_dielect = terdic;

            }
        }

        if (strcmp(argv[x], "-tercon") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0] && argv[z][0] != '-') {

                sscanf(argv[z], "%lf", &tercon);

                LR.sgm_conductivity = tercon;

            }
        }

        if (strcmp(argv[x], "-hp") == 0) {
            // Horizontal polarisation (0)
            LR.pol = 0;
        }

        if (strcmp(argv[x], "-dbg") == 0) {
            debug = 1;
        }


         /*UDT*/ if (strcmp(argv[x], "-udt") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0]) {
                strncpy(udt_file, argv[z], 253);
            }
        }

        /*Prop model */

        if (strcmp(argv[x], "-pm") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0]) {
                sscanf(argv[z], "%d", &propmodel);
            }
        }
        // Prop model variant eg. urban/suburban
        if (strcmp(argv[x], "-pe") == 0) {
            z = x + 1;

            if (z <= y && argv[z][0]) {
                sscanf(argv[z], "%d", &pmenv);
            }
        }
        //Knife edge diffraction
        if (strcmp(argv[x], "-ked") == 0) {
            z = x + 1;
            knifeedge = 1;
        }

        //Normalise Path Profile chart
        if (strcmp(argv[x], "-ng") == 0) {
            z = x + 1;
            normalise = 1;
        }
        //Halve the problem
        if (strcmp(argv[x], "-haf") == 0) {
            z = x + 1;
            if (z <= y && argv[z][0]) {
                sscanf(argv[z], "%d", &haf);
            }
        }

        //Disable threads
        if (strcmp(argv[x], "-nothreads") == 0) {
            z = x + 1;
            use_threads = false;
        }
    }

    /* ERROR DETECTION */
    if (tx_site[0].lat > 90 || tx_site[0].lat < -90) {
        fprintf(stdout,
            "ERROR: Either the lat was missing or out of range!");
        exit(0);

    }
    if (tx_site[0].lon > 360 || tx_site[0].lon < 0) {
        fprintf(stdout,
            "ERROR: Either the lon was missing or out of range!");
        exit(0);

    }
    if (LR.frq_mhz < 20 || LR.frq_mhz > 100000) {
        fprintf(stdout,
            "ERROR: Either the Frequency was missing or out of range!");
        exit(0);
    }
    if (LR.erp > 500000000) {
        fprintf(stdout, "ERROR: Power was out of range!");
        exit(0);

    }
    if (LR.eps_dielect > 80 || LR.eps_dielect < 0.1) {
        fprintf(stdout, "ERROR: Ground Dielectric value out of range!");
        exit(0);

    }
    if (LR.sgm_conductivity > 0.01 || LR.sgm_conductivity < 0.000001) {
        fprintf(stdout, "ERROR: Ground conductivity out of range!");
        exit(0);

    }

    if (tx_site[0].alt < 0 || tx_site[0].alt > 60000) {
        fprintf(stdout,
            "ERROR: Tx altitude above ground was too high: %f",
            tx_site[0].alt);
        exit(0);
    }
    if (altitudeLR < 0 || altitudeLR > 60000) {
        fprintf(stdout,
            "ERROR: Rx altitude above ground was too high!");
        exit(0);
    }

    if(!lidar){
        if (ippd < 300 || ippd > 10000) {
            fprintf(stdout, "ERROR: resolution out of range!");
            exit(0);
        }
    }

    if (contour_threshold < -200 || contour_threshold > 200) {
        fprintf(stdout,
            "ERROR: Receiver threshold out of range (-200 / +200)");
        exit(0);
    }
    if (propmodel > 2 && propmodel < 7 && LR.frq_mhz < 150) {
        fprintf(stdout,
            "ERROR: Frequency too low for Propagation model");
        exit(0);
    }

    if (metric) {
        altitudeLR /= METERS_PER_FOOT;	/* 10ft * 0.3 = 3.3m */
        max_range /= KM_PER_MILE;	/* 10 / 1.6 = 7.5 */
        altitude /= METERS_PER_FOOT;
        tx_site[0].alt /= METERS_PER_FOOT;	/* Feet to metres */
        tx_site[1].alt /= METERS_PER_FOOT;	/* Feet to metres */
        clutter /= METERS_PER_FOOT;	/* Feet to metres */
    }

    /* Ensure a trailing '/' is present in sdf_path */

    if (sdf_path[0]) {
        x = strlen(sdf_path);

        if (sdf_path[x - 1] != '/' && x != 0) {
            sdf_path[x] = '/';
            sdf_path[x + 1] = 0;
        }
    }

    x = 0;
    y = 0;

    min_lat = 70;
    max_lat = -70;


    min_lon = (int)floor(tx_site[0].lon);
    max_lon = (int)floor(tx_site[0].lon);

    txlat = (int)floor(tx_site[0].lat);
    txlon = (int)floor(tx_site[0].lon);

    if (txlat < min_lat)
        min_lat = txlat;

    if (txlat > max_lat)
        max_lat = txlat;

    if (LonDiff(txlon, min_lon) < 0.0)
        min_lon = txlon;

    if (LonDiff(txlon, max_lon) >= 0.0)
        max_lon = txlon;

    if (ppa == 1) {
        rxlat = (int)floor(tx_site[1].lat);
        rxlon = (int)floor(tx_site[1].lon);

        if (rxlat < min_lat)
            min_lat = rxlat;

        if (rxlat > max_lat)
            max_lat = rxlat;

        if (LonDiff(rxlon, min_lon) < 0.0)
            min_lon = rxlon;

        if (LonDiff(rxlon, max_lon) >= 0.0)
            max_lon = rxlon;
    }

    /* Load the required tiles */
    if(lidar){
        int err;

        err = loadLIDAR(lidar_tiles);
        if (err) {
            fprintf(stdout, "Couldn't find one or more of the "
                "lidar files. Please ensure their paths are "
                "correct and try again.\n");
            exit(EXIT_FAILURE);
        }


        if(debug){
            fprintf(stdout,"%.4f,%.4f,%.4f,%.4f,%d x %d\n",max_north,min_west,min_north,max_west,width,height);
        }
        ppd=rint(height / (max_north-min_north));
        yppd=rint(width / (max_west-min_west));

        //ppd=(double)ippd;
        //yppd=ppd;


        if(delta>0){
            tx_site[0].lon+=delta;
        }

    }else{
        // DEM first
        LoadTopoData(max_lon, min_lon, max_lat, min_lat);

            if (area_mode || topomap) {
            for (z = 0; z < txsites && z < max_txsites; z++) {
                /* "Ball park" estimates used to load any additional
                   SDF files required to conduct this analysis. */

                tx_range =
                    sqrt(1.5 *
                     (tx_site[z].alt + GetElevation(tx_site[z])));

                if (LRmap)
                    rx_range = sqrt(1.5 * altitudeLR);
                else
                    rx_range = sqrt(1.5 * altitude);

                /* deg_range determines the maximum
                   amount of topo data we read */

                deg_range = (tx_range + rx_range) / 57.0;

                /* max_range regulates the size of the
                   analysis.  A small, non-zero amount can
                   be used to shrink the size of the analysis
                   and limit the amount of topo data read by
                   ss  A large number will increase the
                   width of the analysis and the size of
                   the map. */

                if (max_range == 0.0)
                    max_range = tx_range + rx_range;

                deg_range = max_range / 57.0;

                // No more than 8 degs
                deg_limit = 3.5;

                if (fabs(tx_site[z].lat) < 70.0)
                    deg_range_lon =
                        deg_range / cos(DEG2RAD * tx_site[z].lat);
                else
                    deg_range_lon = deg_range / cos(DEG2RAD * 70.0);

                /* Correct for squares in degrees not being square in miles */

                if (deg_range > deg_limit)
                    deg_range = deg_limit;

                if (deg_range_lon > deg_limit)
                    deg_range_lon = deg_limit;

                nortRxHin = (int)floor(tx_site[z].lat - deg_range);
                nortRxHax = (int)floor(tx_site[z].lat + deg_range);

                west_min = (int)floor(tx_site[z].lon - deg_range_lon);

                while (west_min < 0)
                    west_min += 360;

                while (west_min >= 360)
                    west_min -= 360;

                west_max = (int)floor(tx_site[z].lon + deg_range_lon);

                while (west_max < 0)
                    west_max += 360;

                while (west_max >= 360)
                    west_max -= 360;

                if (nortRxHin < min_lat)
                    min_lat = nortRxHin;

                if (nortRxHax > max_lat)
                    max_lat = nortRxHax;

                if (LonDiff(west_min, min_lon) < 0.0)
                    min_lon = west_min;

                if (LonDiff(west_max, max_lon) >= 0.0)
                    max_lon = west_max;
            }

            /* Load any additional SDF files, if required */

            LoadTopoData(max_lon, min_lon, max_lat, min_lat);
        }
        ppd=(double)ippd;
        yppd=ppd;
        width = (unsigned)(ippd * ReduceAngle(max_west - min_west));
        height = (unsigned)(ippd * ReduceAngle(max_north - min_north));
    }

    dpp = 1 / ppd;
    mpi = ippd-1;

    // User defined clutter file
    LoadUDT(udt_file);

    // Enrich with Clutter
    if(strlen(clutter_file) > 1){
        /*
        Clutter tiles cover 16 x 12 degs but we only need a fraction of that area.
        Limit by max_range / miles per degree (at equator)
        */
        loadClutter(clutter_file,max_range/45,tx_site[0]);
    }

    if (ppa == 0) {
        if (propmodel == 2) {
            PlotLOSMap(tx_site[0], altitudeLR, ano_filename, use_threads);
            DoLOS(mapfile, geo, kml, ngs, tx_site, txsites);
        } else {
            // 90% of effort here
            PlotPropagation(tx_site[0], altitudeLR, ano_filename,
                    propmodel, knifeedge, haf, pmenv, use_threads);

                        if(debug)
                            fprintf(stdout,"Finished PlotPropagation()\n");

            if(!lidar){
                if (LR.erp == 0.0)
                    hottest=9; // 9dB nearfield

                // nearfield bugfix
                for (lat = tx_site[0].lat - 0.001;
                     lat <= tx_site[0].lat + 0.001;
                     lat = lat + 0.0001) {
                    for (lon = tx_site[0].lon - 0.001;
                         lon <= tx_site[0].lon + 0.001;
                         lon = lon + 0.0001) {
                        PutSignal(lat, lon, hottest);
                    }
                }

            }

            // Write bitmap
            if (LR.erp == 0.0)
                DoPathLoss(mapfile, geo, kml, ngs, tx_site,
                       txsites);
            else if (dbm)
                DoRxdPwr(mapfile, geo, kml, ngs, tx_site,
                     txsites);
            else
                DoSigStr(mapfile, geo, kml, ngs, tx_site,
                     txsites);

        }
        if(lidar){
            east=eastoffset;
            west=westoffset;
        }

        // Print WGS84 bounds
        fprintf(stdout, "|%.6f", north);
        fprintf(stdout, "|%.6f", east);
        fprintf(stdout, "|%.6f", south);
        fprintf(stdout, "|%.6f|", west);

    } else {
        strncpy(tx_site[0].name, "Tx", 3);
        strncpy(tx_site[1].name, "Rx", 3);
        PlotPath(tx_site[0], tx_site[1], 1);
        PathReport(tx_site[0], tx_site[1], tx_site[0].filename, 0,
               propmodel, pmenv, rxGain);
        SeriesData(tx_site[1], tx_site[0], tx_site[0].filename, 1,
               normalise);
    }
    fflush(stdout);

    return 0;
}



// geosun-gnss2kml.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "kml.h"

#ifndef PI
#define	PI 3.14159265358979
#endif

#define ae_WGS84 6378137.0
#define finv_WGS84 298.257223563

static void xyz2blh(const double* xyz, double* blh)
{
	// ecef xyz => blh
	double a = ae_WGS84, finv = finv_WGS84;
	double f = 1.0 / finv, e2 = 2 * f - f * f;
	double x = xyz[0], y = xyz[1], z = xyz[2], lat, lon, ht;
	double R = sqrt(x * x + y * y + z * z);
	double ang = atan(fabs(z / sqrt(x * x + y * y))) * ((z < 0.0) ? -1.0 : 1.0);
	//if (z<0.0) ang = -ang;
	double lat1 = ang;
	double Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
	double Rn = 0.0;
	lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
	if (z < 0.0) lat = -lat;
	while (fabs(lat - lat1) > 1e-12)
	{
		lat1 = lat;
		Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
		lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
		if (z < 0.0) lat = -lat;
	}
	if (lat > PI) lat = lat - 2.0 * PI;
	if (fabs(x) < 1e-12) { if (y >= 0.0) lon = PI / 2.0; else lon = 3.0 * PI / 2.0; }
	else
	{
		lon = atan(fabs(y / x));
		if (x > 0.0) { if (y >= 0.0) lon = lon; else lon = 2.0 * PI - lon; }
		else { if (y >= 0.0) lon = PI - lon; else lon = PI + lon; }
	}
	Rw = sqrt(1 - e2 * sin(lat) * sin(lat));
	Rn = a / Rw;
	ht = R * cos(ang) / cos(lat) - Rn;
	if (lon > PI) lon = lon - 2.0 * PI;
	blh[0] = lat;
	blh[1] = lon;
	blh[2] = ht;
	return;
}


static void geosun_gnss2kml(const char* fname)
{
	FILE* fdat = NULL;
	FILE* fpkml = NULL;

	char fileName[255] = { 0 };
	char out_kml_fpath[255] = { 0 };

	fdat = fopen(fname, "r"); if (fdat == NULL) return;

	strcpy(fileName, fname);
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(out_kml_fpath, "%s.kml", fileName); fpkml = fopen(out_kml_fpath, "w");

	if (fpkml != NULL) print_kml_heder(fpkml);

	double heading = 0;

	char buffer[1024] = { 0 };

	while (!feof(fdat))
	{
		fgets(buffer, sizeof(buffer), fdat);

		if (strlen(buffer) <= 0) continue;

		double blh[3] = { 0 };
		int wn = 0;
		double ws = 0.0;
		int solType = 4;
		char sol_status[255] = { 0 };

		double data[15] = { 0 };
		double vel[3] = { 0 };

		int num = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", data+0, data + 1, data + 2, data + 3, data + 4, data + 5, data + 6, data + 7, data + 8, data + 9, data + 10, data + 11, data + 12, data + 13, data + 14);

		if (num == 8)
		{
			ws = data[1];
			xyz2blh(data + 2, blh);
			blh[0] *= 180.0 / PI;
			blh[1] *= 180.0 / PI;
			heading = data[7];
		}
		else if (num == 15)
		{
			ws = data[1];
			blh[0] = data[3];
			blh[1] = data[2];
			blh[2] = data[4];
			vel[0] = data[8];
			vel[1] = data[9];
			vel[2] = data[10];
			double speed = sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);

			if (speed > 1.0)
			{
				heading = atan2(vel[1], vel[0]);
				heading *= 180.0 / PI;
			}
		}
		else
			continue;


		print_kml_gga(fpkml, blh[0], blh[1], blh[2], solType, ws, heading, sol_status);

	}

	if (fpkml != NULL) print_kml_eof(fpkml);

	if (fdat != NULL) fclose(fdat);
	if (fpkml != NULL) fclose(fpkml);
}

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("%s filename\r\n", argv[0]);
		//geosun_gnss2kml("C:\\geosun\\TRF_LiDAR_042221\\POS\\gnss.txt");
		geosun_gnss2kml("C:\\geosun\\TRF_LiDAR_042221\\POS\\ins.txt");
	}
	else
	{
		geosun_gnss2kml(argv[1]);
	}
}

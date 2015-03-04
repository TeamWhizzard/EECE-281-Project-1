#include "Arduino.h"
#ifndef lab2constants
#define lab2constants

/***************************
 * Musical Constants	   *
 ***************************/

// Pitch frequencies
#define NS      0 // rest
#define NB1     62
#define NC2     65
#define NC2	65
#define NCS2	69
#define ND2	73
#define NDS2	78
#define NE2	82
#define NF2	87
#define NFS2	93
#define NG2	98
#define NGS2	104
#define NA2	110
#define NAS2	117
#define NB2	123
#define NC3	131
#define NCS3 	139
#define ND3	147
#define NDS3	156
#define NE3	165
#define NF3	175
#define NFS3 	185
#define NG3	196
#define NGS3 	208
#define NA3	220
#define NAS3 	233
#define NB3	247
#define NC4	262
#define NCS4 	277
#define ND4	294
#define NDS4 	311
#define NE4	330
#define NF4	349
#define NFS4 	370
#define NG4	392
#define NGS4 	415
#define NA4	440
#define NAS4 	466
#define NB4	494
#define NC5	523
#define NCS5 	554
#define ND5	587
#define NDS5 	622
#define NE5	659
#define NF5	698
#define NFS5 	740
#define NG5	784
#define NGS5 	831
#define NA5	880
#define NAS5 	932
#define NB5	988

// everything will be broken up into 64th notes so onset space is one 64th note

  // NS is onset space
  const int theme_melody[] = {
	// measure one
	  NB3, NB3, NB3, NS,
	  NB4, NB4, NB4, NS,
	  NFS4, NFS4, NFS4, NS,
	  NDS4, NDS4, NDS4, NS,

	  NB4, NS,
	  NFS4, NFS4, NFS4, NFS4, NFS4, NS,
	  NDS4, NDS4, NDS4, NDS4, NDS4, NDS4, NDS4, NS,

	  NC4, NC4, NC4, NS,
	  NC5, NC5, NC5, NS,
	  NG4, NG4, NG4, NS,
	  NE4, NE4, NE4, NS,

	  NC5, NS,
	  NG4, NG4, NG4, NG4, NG4, NS,
	  NE4, NE4, NE4, NE4, NE4, NE4, NE4, NS
  };

  const int theme_bass[] = {
	// measure one
	  NB1, NB1, NB1, NB1,
	  NB1, NB1, NB1, NB1,
	  NB1, NB1, NB1, NS,
	  NB2, NB2, NB2, NS,

	  NB1, NB1, NB1, NB1,
	  NB1, NB1, NB1, NB1,
	  NB1, NB1, NB1, NS,
	  NB2, NB2, NB2, NS,

	  NC2, NC2, NC2, NC2,
	  NC2, NC2, NC2, NC2,
	  NC2, NC2, NC2, NS,
	  NC3, NC3, NC3, NS,

	  NC2, NC2, NC2, NC2,
	  NC2, NC2, NC2, NC2,
	  NC2, NC2, NC2, NS,
	  NC3, NC3, NC3, NS
  };

#endif

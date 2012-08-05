
//wingrime 2012 (c)
//license GPL
//Battary voltage to procentage convection routine 
//
#include <linux/kernel.h>
#include <linux/string.h>
typedef struct charging_table {
	u32 vol;
	u32 capacity;
} charging_table;
//10 procent battary
charging_table tbl[] = {
	
  {	3200,	0       },
  {	3303,   1       },
  {     3374,   2       },
  {	3439,   3       },
  {	3482,   4       },
  {	3507,   5       },
  {	3522,   6	},
  {	3532,	7	},
  {	3541,	8	},
  {	3548,	9	},
  {	3557,	10	},
  {	3567,	11	},
  {	3577,	12	},
  {	3585,	13	},
  {	3593,	14	},
  {	3600,	15	},
  {	3637,   20	},
  {	3683,   30	},
  {	3729,   40	},
  {	3775,   50	},
  {	3825,   60   	},
  {	3875,   70      },
  {	3925,   80      },
  {	3975,   90      },
  {	3985,   91      },
  {	3995,   92     	},
  {	4005,   93 	},
  {	4015,   94 	},
  {	4025,   95 	},
  {	4035,   96 	},
  {	4045,   97 	},
  {	4055,   98 	},
  {	4065,   99 	},
  {	4199,   99	}
};

u32 calculate_capacity(u32 v)
{
	int i;
	u32 cap;
	
	printk("%s: batt_vol=%d\n",__func__,v);
	cap = 0; 
	for(i=0;i<ARRAY_SIZE(tbl);i++){
		if(v<=3200){
			cap=0;
			break;
		}
		if(v>=4200){
			cap=100;
			break;
		}
		if(v>=tbl[i].vol){
			if(i==(ARRAY_SIZE(tbl)-1)){
				cap=99;
				break;
			}
			continue;
		}
		cap=tbl[i].capacity;
		break;
	}
	printk("%s: capacity=%d\n",__func__,cap);

	return cap;
}

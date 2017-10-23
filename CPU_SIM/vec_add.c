

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <CL/opencl.h>

#include <sys/time.h>
#include <time.h>

#define VEC_SIZE 10000


////////////////////////////////////////////////////////////////////////////////
typedef struct timespec timespec;
timespec diff(timespec start, timespec end);
timespec sum(timespec t1, timespec t2);
void printTimeSpec(timespec t);
timespec tic( );
void toc( timespec* start_time );
double get_current_msec( );

timespec diff(timespec start, timespec end)
{   
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

timespec sum(timespec t1, timespec t2) {
	timespec temp;
	if (t1.tv_nsec + t2.tv_nsec >= 1000000000) {
		temp.tv_sec = t1.tv_sec + t2.tv_sec + 1;
		temp.tv_nsec = t1.tv_nsec + t2.tv_nsec - 1000000000;
	} else {
		temp.tv_sec = t1.tv_sec + t2.tv_sec;
		temp.tv_nsec = t1.tv_nsec + t2.tv_nsec;
	}
	return temp;
}

void printTimeSpec(timespec t) {
	printf("elapsed time: %d.%09d\n", (int)t.tv_sec, (int)t.tv_nsec);
}

timespec tic( )
{
	timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

void toc( timespec* start_time )
{
	timespec current_time;
	clock_gettime(CLOCK_REALTIME, &current_time);
	printTimeSpec( diff( *start_time, current_time ) );
	*start_time = current_time;
}

double get_current_msec( )
{
	timespec current_time;
	clock_gettime(CLOCK_REALTIME, &current_time);
	return current_time.tv_nsec/1e6+current_time.tv_sec*1e3;
}

void sw_extend(unsigned short qs_baddr, int *qs, unsigned short ts_baddr, short qlen, short tlen, char o_ins,
			   char e_ins, char o_del, char e_del, char penClip, char w_in, char h0, short *regScore, short qBeg, short max_ins, short max_del,
			   short *w_ret, short *qle_ret, short *tle_ret, short *gtle_ret, short *gscore_ret, short *maxoff_ret)
{
	//#pragma HLS INLINE

	int i, j;
	int k, l;
	short max_i, max_ie, max_off;
	short gscore;
	char max_j;
	char oe_del = o_del + e_del;
	char oe_ins = o_ins + e_ins;
	short beg, end;
	char backw_tmp=0;
	char backw_reg=0;
	char forw_update=0;
	char forw_tmp=0;
	char forw_reg=0;
	short abs_mj_m_i;
	char tmp_ehh_m_eins;
	char tmp_eme;
	char h1_init_val;
	char max;
	char h, e;
	char e_tmp;
	char h_tmp;
	char h1_reg;
	char t, f = 0, h1, m = 0;
	char mj = -1;
	char q_i = 0, q_j = 0;
	short prev;
	char isBreak;
	char aw1;
	char aw_tmp;
	char h0_arr[2];
#pragma HLS ARRAY_PARTITION variable=h0_arr complete dim=0
	//	short sc0;
	//	short h0;

	const char my_mat[5][5]={{1, -4, -4, -4, -1}, {-4, 1, -4, -4, -1}, {-4, -4, 1, -4, -1}, {-4, -4, -4, 1, -1}, {-1, -1, -1, -1, -1}};
#pragma HLS ARRAY_PARTITION variable=my_mat complete dim=0
	char eh_h [256];
#pragma HLS ARRAY_MAP variable=eh_h instance=eh_arr vertical
#pragma HLS RESOURCE variable=eh_h core=RAM_2P_BRAM
	char eh_e [256];
#pragma HLS ARRAY_MAP variable=eh_e instance=eh_arr vertical
#pragma HLS RESOURCE variable=eh_e core=RAM_2P_BRAM

	//	sc0 = *regScore;
	//	h0_arr[0] = h0_ori;
	//	h0_arr[1] = sc0;
	//	qle = -1;
	//	tle = -1;
	//	gtle = -1;
	max = h0;
	max_i = max_j = -1;
	max_ie = -1;
	gscore = -1;
	max_off = 0;

	k = 0;
	isBreak = 0;
ext_while_loop : while ((k < 2) && (!isBreak))
				 {
#pragma HLS LOOP_TRIPCOUNT min=2 max=2
					 prev = *regScore;
					 aw_tmp = w_in << k;
					 aw1 = aw_tmp < max_ins ? aw_tmp : max_ins;
					 aw1 = aw1 < max_del ? aw1 : max_del;
					 beg = 0;
					 end = qlen;
					 if (h0 > oe_ins) {
						 tmp_eme = h0 - oe_ins;
					 }
					 else {
						 tmp_eme = 0;
					 }
					 h1_init_val = h0 - o_del;
target_loop : for (i = 0; i < tlen; i++) {
					 f = 0; m = 0; mj = -1;
					 //while(pe_seeds.empty());
					 //			q_i = ts[ts_baddr + i];
					 q_i = qs[ts_baddr + i];
					 h1_init_val -= e_del;
					 h1 = h1_init_val;
					 if (h1 < 0) h1 = 0;
					 if (beg < i - aw1) beg = i - aw1;
					 if (end > i + aw1 + 1) end = i + aw1 + 1;
					 if (end > qlen) end = qlen;
					 backw_tmp = 0; backw_reg = 0;
					 forw_tmp = 0; forw_reg = 0;
					 forw_update = 0;
query_loop : for (j = beg; j < end; ++j) {
#pragma HLS LOOP_TRIPCOUNT min=50 max=50
#pragma AP pipeline II=1
#pragma AP dependence variable=eh_e array inter false
#pragma AP dependence variable=eh_h array inter false
					 q_j = qs[qs_baddr + j];
					 h_tmp = eh_h[j];// get H(i-1,j-1) and E(i-1,j)
					 e_tmp = eh_e[j];
					 if (i == 0) {
						 e = 0;
						 if (j == 0) {
							 h = h0;
						 }
						 else if (j == 1) {
							 h = tmp_eme;
						 }
						 else {
							 tmp_eme -= e_ins;
							 if (tmp_eme > 0) {
								 h = tmp_eme;
							 }
							 else {
								 h = 0;
							 }
						 }
					 }
					 else {
						 e = e_tmp;
						 h = h_tmp;
					 }
					 h1_reg = h1;
					 h += my_mat[q_i][q_j];
					 h = h > e? h : e;
					 h = h > f? h : f;
					 h1 = h;             // save H(i,j) to h1 for the next column
					 if (h1_reg == 0) {
						 backw_tmp = 0;
					 }
					 else {
						 backw_tmp++;
					 }
					 if (m <= h)
					 {
						 mj = j;
						 m = h;
						 backw_reg = backw_tmp;
					 }
					 if (j >= mj+2) {
						 if (forw_update == 0) { //((h1_reg == 0) &&
							 if (h1_reg == 0) {
								 forw_update = 1;
							 }
							 else {
								 forw_tmp++;
							 }
						 }
					 }
					 else {
						 forw_tmp = 0;
						 forw_update = 0;
					 }
					 t = h - oe_del;
					 t = t > 0? t : 0;
					 e -= e_del;
					 e = e > t? e : t;   // computed E(i+1,j)
					 eh_e[j] = e; // save E(i+1,j) for the next row
					 eh_h[j] = h1_reg;          // set H(i,j-1) for the next row
					 t = h - oe_ins;
					 t = t > 0? t : 0;
					 f -= e_ins;
					 f = f > t? f : t;   // computed F(i,j+1)
			 }
			 eh_h[end] = h1;
			 eh_e[end] = 0;
			 if ((forw_update == 0) && (h1 != 0)) {
				 if ((j >= mj+2) || (forw_tmp != 0)) {
					 forw_tmp++;
				 }
			 }
			 //			fprintf(fp_test, "\t k = %d;\t i = %d;\t j = %d;\t beg = %d;\t end = %d;\t gscore = %d;\t h1 = %d;\t max_ie = %d;\t", k, i, j, beg, end, gscore, h1, max_ie);
			 if (j == qlen) {
				 max_ie = gscore > h1? max_ie : i;
				 gscore = gscore > h1? gscore : h1;
			 }
			 //			fprintf(fp_test, "\t m = %d;\t max = %d;\t mj = %d;\t max_j = %d;\t\n ", m, max, mj, max_j);
			 if (m == 0) break;
			 if (m > max) {
				 max = m; max_i = i; max_j = mj;
				 if (mj >= i) abs_mj_m_i = mj - i;
				 else abs_mj_m_i = i - mj;
				 if (max_off < abs_mj_m_i) max_off = abs_mj_m_i;
			 }
			 // update beg and end for the next round
			 //for (j = mj; j >= beg && eh_h[j]; --j);
			 //beg = j + 1;
			 //for (j = mj + 2; j <= end && eh_h[j]; ++j);
			 //end = j;
			 j = mj - backw_reg;
			 beg = j + 1;
			 j = mj + 2 + forw_tmp;
			 end = j;
			  }
			  *qle_ret = max_j + 1;
			  *tle_ret = max_i + 1;
			  *gtle_ret = max_ie + 1;
			  *gscore_ret = gscore;
			  *maxoff_ret = max_off;
			  *regScore = max;
			  if (max == prev || ( max_off < (aw_tmp >> 1) + (aw_tmp >> 2))) isBreak = 1;
			  k++;
				 }
				 *w_ret = aw_tmp;
				 //	*score = max;
}

int main()
{   
//    int i;

//    int inc = 0;
	int size = 0;
	FILE *f_inp = fopen("total_input.dat", "rb");
	FILE *f_outp = fopen("results.dat", "wb");
	if (f_inp == NULL) 
	{ 
		return -1; // -1 means file opening fail 
	} 
	fseek(f_inp, 0, SEEK_END);
	size = ftell(f_inp);
	fseek(f_inp, 0, SEEK_SET);
	printf("size = %d\n", size);

	int * a; //cmost_malloc_1d( &a, "total_input.dat" , 4, 409600);
	a = (int *)calloc(409600, sizeof(int));
	fread(a, sizeof(int), size, f_inp);

    //int * b; cmost_malloc_1d( &b, "int_i2.dat", 4, VEC_SIZE);
    int * c; //cmost_malloc_1d( &c, "zero.dat" , 4, 8192);
	c = (int *)calloc(16384, sizeof(int));
    if (!(a && c)) while(1);

#pragma cmost graph_begin
#pragma cmost task_begin name="vec_add" lib="true"

	// Get the start time
	timespec timer = tic( );
//    {
		int ii = 0;
		int idx = 0;
		char o_del;
		char e_del;
		char o_ins;
		char e_ins;
		char penClip[2];
#pragma HLS ARRAY_PARTITION variable=penClip complete dim=0
		char w_in;
		int taskNums;
		int tmp_compar;
		tmp_compar = a[0];
		o_del = tmp_compar & 0xFF; //6
		e_del = (tmp_compar >> 8) & 0xFF; //1
		o_ins = (tmp_compar >> 16) & 0xFF; //6
		e_ins = (tmp_compar >> 24) & 0xFF; //1
		tmp_compar = a[1];
		penClip[0] = tmp_compar & 0xFF; //100
		penClip[1] = (tmp_compar >> 8) & 0xFF;
		w_in = (tmp_compar >> 16) & 0xFF;
		//h0 = (tmp_compar >> 24) & 0xFF; //45
		taskNums = a[2];

		for (ii=0; ii<taskNums; ii++)
		{
			int seed_index;
			short qlen[2];
#pragma HLS ARRAY_PARTITION variable=qlen complete dim=0
			short tlen[2];
#pragma HLS ARRAY_PARTITION variable=tlen complete dim=0
			short max_ins[2];
#pragma HLS ARRAY_PARTITION variable=max_ins complete dim=0
			short max_del[2];
#pragma HLS ARRAY_PARTITION variable=max_del complete dim=0
			char h0;
			short regScore;
			short qBeg_ori;

			short qle;
			short tle;
			short gtle;
			short gscore;
			short maxoff;
			short qBeg;
			short rBeg;
			short qEnd;
			short rEnd;
			short score;
			short trueScore;
			short width;

			unsigned char i;
			short k, l;
			short qlen2;
			short tlen2;
			unsigned short qs_baddr;
			unsigned short ts_baddr;
			short aw[2];
#pragma HLS ARRAY_PARTITION variable=aw complete dim=0
			int tmp_parame;
			int qr_offset;
			short sc0;
			short h0_arr[2];
#pragma HLS ARRAY_PARTITION variable=h0_arr complete dim=0

			int qrLen_div8;
			int tmp_qr_data;
			int tmp_query_mem[256];
#pragma HLS RESOURCE variable=tmp_query_mem core=RAM_2P_BRAM
			int query_mem[2048];
#pragma HLS RESOURCE variable=query_mem core=RAM_2P_BRAM
			//		int target_mem[2048];
			//	#pragma HLS RESOURCE variable=target_mem core=RAM_2P_BRAM
			int param_mem[8];

			//memcpy(param_mem, &a[(ii+1)*8], (8)*4);
			for (idx=0; idx<8; idx++)
			{
				param_mem[idx] = a[(ii+1)*8 + idx];
			}
			//tmp_parame = totalinp[ii*8 + 0];
			//while(pe_seeds.empty());
			//pe_seeds.read(seed_index);
			//tmp_parame = totalinp[ii*8 + 1];
			//while(pe_seeds.empty());
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[(ii+1)*8 + 0];
			tmp_parame = param_mem[0];
			qlen[0] = tmp_parame & 0xFFFF; //55
			tlen[0] = (tmp_parame >> 16) & 0xFFFF; //105
			//tmp_parame = totalinp[ii*8 + 2];
			//while(pe_seeds.empty());
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[(ii+1)*8 + 1];
			tmp_parame = param_mem[1];
			qlen[1] = tmp_parame & 0xFFFF; //55
			tlen[1] = (tmp_parame >> 16) & 0xFFFF; //105
			//		tmp_parame = totalinp[(ii+1)*8 + 2];
			tmp_parame = param_mem[2];
			qr_offset = tmp_parame;
			//		tmp_parame = totalinp[(ii+1)*8 + 3];
			tmp_parame = param_mem[3];
			regScore = tmp_parame & 0xFFFF; //100
			qBeg_ori = (tmp_parame >> 16) & 0xFFFF;
			//		tmp_parame = totalinp[(ii+1)*8 + 4];
			tmp_parame = param_mem[4];
			h0 = tmp_parame & 0xFFFF;
			//max_del[0] = (tmp_parame >> 16) & 0xFFFF;
			//tmp_parame = totalinp[ii*8 + 3];
			//while(pe_seeds.empty());
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[ii*8 + 2];
			//		o_del = tmp_parame & 0xFF; //6
			//		e_del = (tmp_parame >> 8) & 0xFF; //1
			//		o_ins = (tmp_parame >> 16) & 0xFF; //6
			//		e_ins = (tmp_parame >> 24) & 0xFF; //1
			//tmp_parame = totalinp[ii*8 + 4];
			//while(pe_seeds.empty());
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[ii*8 + 3];
			//		penClip[0] = tmp_parame & 0xFF; //100
			//		penClip[1] = (tmp_parame >> 8) & 0xFF;
			//		w_in = (tmp_parame >> 16) & 0xFF;
			//		h0 = (tmp_parame >> 24) & 0xFF; //45
			//tmp_parame = totalinp[ii*8 + 5];
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[(ii+1)*8 + 5];
			tmp_parame = param_mem[5];
			max_ins[0] = tmp_parame & 0xFFFF;
			max_del[0] = (tmp_parame >> 16) & 0xFFFF;
			//		tmp_parame = totalinp[(ii+1)*8 + 6];
			tmp_parame = param_mem[6];
			max_ins[1] = tmp_parame & 0xFFFF;
			max_del[1] = (tmp_parame >> 16) & 0xFFFF;
			//tmp_parame = totalinp[ii*8 + 6];
			//while(pe_seeds.empty());
			//pe_seeds.read(tmp_parame);
			//		tmp_parame = totalinp[(ii+1)*8 + 6];
			//		regScore = tmp_parame & 0xFFFF; //100
			//		qBeg_ori = (tmp_parame >> 16) & 0xFFFF;
			//tmp_parame = totalinp[ii*8 + 7];
			//while(pe_seeds.empty());
			//pe_seeds.read(qr_offset);
			//		tmp_parame = totalinp[(ii+1)*8 + 7];
			//		qr_offset = tmp_parame;

			aw[0] = w_in;
			aw[1] = w_in;
			qBeg = 0;
			qEnd = qlen[1];
			rBeg = 0;
			rEnd = 0;
			trueScore = regScore;
			qle = -1;
			tle = -1;
			gtle = -1;
			gscore = -1;
			maxoff = -1;
			qlen2 = qlen[0] + qlen[1];
			tlen2 = tlen[0] + tlen[1];
			qrLen_div8 = qlen2+tlen2;
			if ((qrLen_div8 & 0x00000007) != 0) {
				qrLen_div8 = (qrLen_div8 >> 3) + 1;
			}
			else {
				qrLen_div8 = (qrLen_div8 >> 3);
			}

			//memcpy(tmp_query_mem, &a[qr_offset], (qrLen_div8)*4);
			for (idx=0; idx<qrLen_div8; idx++)
			{
				tmp_query_mem[idx] = a[qr_offset + idx];
			}

			for (k=0; k<qrLen_div8; k++) {
				tmp_qr_data = tmp_query_mem[k];
				for (l=0; l<8; l++) {
					query_mem[k*8 + l] = (tmp_qr_data & 0xF0000000) >> 28;
					tmp_qr_data <<= 4;
				}
			}
			//	load_query : for (k = 0; k < qlen2; k++) {
			//#pragma HLS PIPELINE II=1
			//#pragma HLS LOOP_TRIPCOUNT min=50 max=50
			//			query_mem[k] = totalinp[qr_offset];
			//			qr_offset++;
			//while(pe_seeds.empty());
			//pe_seeds.read(query_data);
			//query_mem[k] = query_data;
			//		}
			//	memcpy(target_mem, &totalinp[qr_offset+qlen2], tlen2*4);
			//	load_target : for (k = 0; k < tlen2; k++) {
			//#pragma HLS PIPELINE II=1
			//#pragma HLS LOOP_TRIPCOUNT min=100 max=100
			//		    target_mem[k] = totalinp[qr_offset];
			//			qr_offset++;
			//while(pe_seeds.empty());
			//pe_seeds.read(target_data);
			//target_mem[k] = target_data;
			//		}
			qs_baddr = 0;
			//		ts_baddr = 0;
			ts_baddr = qlen2;
left_right_loop : for (i=0; i<2; i++)
				  {
					  //#pragma HLS PIPELINE
#pragma HLS LOOP_TRIPCOUNT min=2 max=2
					  sc0 = regScore;
					  h0_arr[0] = h0;
					  h0_arr[1] = sc0;
					  if (qlen[i] > 0) {
						  sw_extend(qs_baddr, query_mem, ts_baddr, qlen[i], tlen[i], o_ins, e_ins, o_del, e_del, penClip[i],
							  w_in, h0_arr[i], &regScore, qBeg_ori, max_ins[i], max_del[i], &aw[i], &qle, &tle, &gtle, &gscore, &maxoff);
						  score = regScore;
						  if (gscore <= 0 || gscore <= (regScore - penClip[i])) {
							  if (i == 0) {
								  qBeg = qBeg_ori - qle;
								  rBeg = -tle;
								  trueScore = regScore;
							  }
							  else {
								  qEnd = qle;
								  rEnd = tle;
								  trueScore += regScore - sc0;
							  }
						  }
						  else {
							  if (i == 0) {
								  qBeg = 0;
								  rBeg = -gtle;
								  trueScore = gscore;
							  }
							  else {
								  qEnd = qlen[1];
								  rEnd = gtle;
								  trueScore += gscore - sc0;
							  }
						  }
					  }
					  qs_baddr += qlen[i];
					  ts_baddr += tlen[i];
				  }
				  if (aw[0] > aw[1]) width = aw[0];
				  else width = aw[1];

				  //		fprintf(fp_test, "\t ii = %d;\t tle = %d;\t gtle = %d;\t rBeg = %d;\n ", ii, tle, gtle, rBeg);
				  //		fprintf(fp_test, "\t ii = %d;\t qBeg_ori = %d;\t qle = %d;\t qBeg = %d;\n ", ii, qBeg_ori, qle, qBeg);
				  //while(pe_matchs.full());
				  //pe_matchs.write(seed_index);
				  c[ii*4 + 0] = (qBeg & 0xFFFF) | ((qEnd<<16) & 0xFFFF0000);
				  //while(pe_matchs.full());
				  //pe_matchs.write(result_data);
				  c[ii*4 + 1] = (rBeg & 0xFFFF) | ((rEnd<<16) & 0xFFFF0000);
				  //while(pe_matchs.full());
				  //pe_matchs.write(result_data);
				  c[ii*4 + 2] = (score & 0xFFFF) | ((trueScore<<16) & 0xFFFF0000);
				  //while(pe_matchs.full());
				  //pe_matchs.write(result_data);
				  c[ii*4 + 3] = width & 0xFFFF;
		}
		//	fclose(fp_test);
//    }

	// Get the execution time
	toc(&timer);

#pragma cmost task_end
#pragma cmost graph_end

    //cmost_dump_1d(c, "results.dat");
	fwrite(c, sizeof(int), taskNums*4, f_outp);
	free(a);
	free(c);
	fclose(f_inp);
	fclose(f_outp);
	f_inp = NULL;
	f_outp = NULL;


    //cmost_free_1d(a);
//    cmost_free_1d(b);
    //cmost_free_1d(c);

    return 0;
}



#ifndef SYSDEP_H_INCLUDED
#include "sysdep1.h"
#endif
#include "stdio.h"
#include "errno.h"
#ifndef NULL
/* ANSI C */
#include "stddef.h"
#endif

#ifndef SEEK_SET
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2
#endif

#ifndef FOPEN
#define FOPEN fopen
#endif

#ifndef FREOPEN
#define FREOPEN freopen
#endif

#ifndef FSEEK
#define FSEEK fseek
#endif

#ifndef FSTAT
#define FSTAT fstat
#endif

#ifndef FTELL
#define FTELL ftell
#endif

#ifndef OFF_T
#define OFF_T long
#endif

#ifndef STAT_ST
#define STAT_ST stat
#endif

#ifndef STAT
#define STAT stat
#endif

#ifdef MSDOS
#ifndef NON_UNIX_STDIO
#define NON_UNIX_STDIO
#endif
#endif

#ifdef UIOLEN_int
typedef int uiolen;
#else
typedef long uiolen;
#endif

/*units*/
typedef struct
{        FILE *ufd;        /*0=unconnected*/
        char *ufnm;
#ifndef MSDOS
        long uinode;
        int udev;
#endif
        int url;        /*0=sequential*/
        flag useek;        /*true=can backspace, use dir, ...*/
        flag ufmt;
        flag urw;        /* (1 for can read) | (2 for can write) */
        flag ublnk;
        flag uend;
        flag uwrt;        /*last io was write*/
        flag uscrtch;
} unit;

#undef Void
#ifdef KR_headers
#define Void /*void*/
extern int (*f__getn)();        /* for formatted input */
extern void (*f__putn)();        /* for formatted output */
extern void x_putc();
extern long f__inode();
extern VOID sig_die();
extern int (*f__donewrec)(), t_putc(), x_wSL();
extern int c_sfe(), err__fl(), xrd_SL(), f__putbuf();
#else
#define Void void
#ifdef __cplusplus
extern "C" {
#endif
extern int (*f__getn)(void);        /* for formatted input */
extern void (*f__putn)(int);        /* for formatted output */
extern void x_putc(int);
extern long f__inode(char*,int*);
extern void sig_die(const char*,int);
extern void f__fatal(int, const char*);
extern int t_runc(alist*);
extern int f__nowreading(unit*), f__nowwriting(unit*);
extern int fk_open(int,int,ftnint);
extern int en_fio(void);
extern void f_init(void);
extern int (*f__donewrec)(void), t_putc(int), x_wSL(void);
extern void b_char(const char*,char*,ftnlen), g_char(const char*,ftnlen,char*);
extern int c_sfe(cilist*), z_rnew(void);
extern int err__fl(int,int,const char*);
extern int xrd_SL(void);
extern int f__putbuf(int);
#endif
extern flag f__init;
extern cilist *f__elist;        /*active external io list*/
extern flag f__reading,f__external,f__sequential,f__formatted;
extern int (*f__doend)(Void);
extern FILE *f__cf;        /*current file*/
extern unit *f__curunit;        /*current unit*/
extern unit f__units[];
#define err(f,m,s) {if(f) errno= m; else f__fatal(m,s); return(m);}
#define errfl(f,m,s) return err__fl((int)f,m,s)

/*Table sizes*/
#define MXUNIT 100

extern int f__recpos;        /*position in current record*/
extern OFF_T f__cursor;        /* offset to move to */
extern OFF_T f__hiwater;        /* so TL doesn't confuse us */
#ifdef __cplusplus
        }
#endif

#define WRITE        1
#define READ        2
#define SEQ        3
#define DIR        4
#define FMT        5
#define UNF        6
#define EXT        7
#define INT        8

#define buf_end(x) (x->_flag & _IONBF ? x->_ptr : x->_base + BUFSIZ)

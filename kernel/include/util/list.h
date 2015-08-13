#ifndef _LIST_H_
#define _LIST_H_

struct list_head {
	struct list_head *next, *prev;
};

#endif // _LIST_H_

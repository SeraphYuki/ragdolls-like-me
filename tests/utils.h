#ifndef UTILS_TABLE
#define UTILS_TABLE

#define TABLE_SIZE 10007 // must be prime

typedef struct Node {
	char *key;
	void *value;
	struct Node *next;
} Node;


typedef struct {
	Node * buckets[TABLE_SIZE];
} HashTable_t;

//typedef struct {
	//int *indicies;
	//char **data;
	//int size;
//} HashTable;

int HashTable_Insert(HashTable_t *table, const char *key, void *value);
HashTable_t *HashTable_Create();
int HashTable_Delete(HashTable_t *table, const char *key);
void HashTable_Free(HashTable_t *table);
const char *HashTable_Search(HashTable_t *table, const char *key);

#endif
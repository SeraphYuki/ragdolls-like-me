#ifndef HASH_TABLE
#define HASH_TABLE

typedef struct {
	int *indicies;
	char **data;
	int size;
} HashTable;

HashTable HashTable_Create(int size);
char *HashTable_Get(HashTable *h, unsigned char *label, int labelsize);
void HashTable_Add(HashTable *h, unsigned char *label, int labelsize, unsigned char *data, int datasize);
void HashTable_Free(HashTable *h);
#endif
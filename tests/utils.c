#include "utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

static unsigned int hash(const char *key){
	unsigned long hash_value = 5381;
	int c;
	while((c = *key++)){
		hash_value = ((hash_value << 5) + hash_value) + c;
	}
	return hash_value % TABLE_SIZE;
}


HashTable_t *HashTable_Create(){
	HashTable_t *table = (HashTable_t*)malloc(sizeof(HashTable_t));
	memset(table, 0, sizeof(HashTable_t));	

	return table;
}

int HashTable_Insert(HashTable_t *table, const char *key, void *value){

	unsigned int index = hash(key);
	Node *curr = table->buckets[index];
	
	while(curr != NULL){
		if(strcmp(curr->key, key) == 0){
			curr->value = value;
			return 1;
		}
		curr = curr->next;
	}
	
	Node *new = (Node*)malloc(sizeof(Node));
	new->key = strdup(key);
	new->value = value;
	
	new->next = table->buckets[index];
	table->buckets[index] = new;
	
	return 1;
}


const char *HashTable_Search(HashTable_t *table, const char *key){
	
	if(!table || !key) return 0;
	
	unsigned int index = hash(key);
	Node *curr = table->buckets[index];
	
	while(curr != NULL){
		
		if(strcmp(curr->key, key) == 0 ) {
			return curr->value;
		}
		
		curr = curr->next;		
	}
	
}

int HashTable_Delete(HashTable_t *table, const char *key){
	
	if(!table || !key) return 0;
	
	unsigned int index = hash(key);
	Node *curr = table->buckets[index];
	Node *prev = NULL;
	
	while(curr != NULL){
		
		if(strcmp(curr->key, key) == 0 ) {
			
				if(prev == NULL){
					table->buckets[index] = curr->next;
				} else {
					prev->next = curr->next;
				}
				if(curr->key) free(curr->key);
				if(curr) free(curr);
				return 1;
			}
		curr = curr->next;		
	}
	return 0;
}

void HashTable_Free(HashTable_t *table){
	
	if(!table) return;
	
	int k;
	for(k = 0; k < TABLE_SIZE; k++){
		Node *curr = table->buckets[k];
		while(curr != NULL){
			Node *tmp = curr;
			curr = curr->next;
			if(tmp->key) free(tmp->key);
			if(tmp) free(tmp);
		}
		
	}
	
	free(table);
}


//HashTable HashTable_Create(int size){
	 //HashTable ret;
	 //ret.indicies = malloc(sizeof(int) * size);
	 //ret.size = size;
	 //return ret;
//}


//static int Hash(HashTable *h, unsigned char *label, int labelsize){
	 //int index = 0;
	 //int seed = 43252;
	 //int i;
	 //for(i = 0; i < labelsize; i++){
	     //index = index * seed + label[i];
	 //}

    //return index % h->size;
//}

//char *HashTable_Get(HashTable *h, unsigned char *label, int size){
	 //int index = Hash(h, label, size);
	 //return h->data[index];
//}

//void HashTable_Add(HashTable *h, unsigned char *label, int labelsize, unsigned char *data, int datasize){

    //int index = Hash(h, label, labelsize);

    //h->data[index] = malloc(datasize);
	 //memcpy(h->data[index], data, datasize);
//}

//void HashTable_Free(HashTable *h){

    //if(h->indicies) free(h->indicies);
	 //int i;
	 //for(i = 0; i < h->size; i++){
	     //if(h->data[i]) free(h->data[i]);
	 //}

//}

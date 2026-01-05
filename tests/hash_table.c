#include "hash_table.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

HashTable HashTable_Create(int size){
    HashTable ret;
    ret.indicies = malloc(sizeof(int) * size);
    ret.size = size;
    return ret;
}


static int Hash(HashTable *h, unsigned char *label, int labelsize){
    int index = 0;
    int seed = 43252;
    int i;
    for(i = 0; i < labelsize; i++){
        index = index * seed + label[i];
    }

    return index % h->size;
}

char *HashTable_Get(HashTable *h, unsigned char *label, int size){
    int index = Hash(h, label, size);
    return h->data[index];
}

void HashTable_Add(HashTable *h, unsigned char *label, int labelsize, unsigned char *data, int datasize){

    int index = Hash(h, label, labelsize);

    h->data[index] = malloc(datasize);
    memcpy(h->data[index], data, datasize);
}

void HashTable_Free(HashTable *h){

    if(h->indicies) free(h->indicies);
    int i;
    for(i = 0; i < h->size; i++){
        if(h->data[i]) free(h->data[i]);
    }

}

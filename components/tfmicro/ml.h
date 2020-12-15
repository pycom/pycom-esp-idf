/*
 * Copyright (c) 2020, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 *
 * This is the exported API for ML module.
 */

#ifndef ML_H_
#define ML_H_

/* 
* From array.h 
*/

#define MAX_NDIM 2

typedef struct Shape 
{
    // Array with size for each dimmension.
    int dimensions[MAX_NDIM]; 

    // Number of dimmensions.
    int ndim; 
} Shape;

typedef struct Array 
{
    // Shape for this Array.
    Shape shape;

    // Push back index. Used by the push_back_value().
    int push_back_index;

    // Buffer to hold the data.
    float* buffer;
    int buffer_size;
} Array;

/* 
* From list.h 
*/

typedef struct Entry 
{
    void *data;
    struct Entry *prev;
    struct Entry *next;
}Entry;

typedef struct List {
    struct Entry *head;
    struct Entry *tail;
    void (*delete_entry_data)(void *data);
} List;

void l_delete_list(List *list);

/* Macro for iterating over a List pointer. */
#define l_for_each_entry(entry, list) for(Entry *entry = (list != NULL) ? (list)->head : NULL; entry != NULL; entry = entry->next)


/* 
* From model.h 
*/

bool new_model(const char *model_definition_str);
List* run_model(float *data, int length);


/* 
* From data_pipeline.h 
*/

// The model returns one ReturnedData instance for every output block.
typedef struct ReturnedData
{
    Array *data;
    char *block_id;
} ReturnedData;


#endif /*ML_H_*/ 

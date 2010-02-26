////////////////////////////////////////////////////////////////////////////////////
// Nom: debug.h
// Description: fichier associé au source debug.c
// Programmeur: GAUCHER Alexis, étudiant isep
// Création: 19 decembre 2007
// Modifications: premiere version
// Remarque: !! Ce fichier n'est compilé qu'en mode DEBUG (#define _DEBUG)
////////////////////////////////////////////////////////////////////////////////////

/* --------------- UTILISATION DE LA LIBRAIRIE DEBUG.H ---------------

Cette librairie permet de detecter des fuites mémoire provenant des malloc.
Pour l'utiliser, l'inclure seulement dans les fichiers sources que l'on
désire tester, et appeler la fonction _closedebug() juste avant la fermeture
du programme.

exemple:

#include <stdio.h>
#include "debug.h"

int main()
{
[...]
_closedebug();
return 0;
}

----------------------------------------------------------------------------- */



// on ne compile qu'en mode DEBUG
#ifdef _DEBUG

// on detourne la fonction malloc() si on est en mode DEBUG
// et on la reoriente vers une fonction maison qui va recuperer
// et mémoriser les informations sur le pointer et la zone alouée
#define malloc(n) _malloc(n, __LINE__, __FILE__)

// idem pour free. On fait passer par une fonction _free() intermediaire
// pour traiter l'information
#define free(p) _free(p, __LINE__, __FILE__);p=NULL

// fonctions utiles définies dans debug.c :
char* GetShortFileName(char* filename);
void* _malloc(size_t n, int line, char* file);
void _free(void* p, int line, char* file);
void _closedebug();


// si le mode DEBUG n'est pas mis
#else
// on fait disparaître du programme la fonction _closedebug() qui n'a
// plus lieu d'être. C'est un peu bidouille.
#define _closedebug() (0)



#endif 

 ////////////////////////////////////////////////////////////////////////////////////
 // Nom: debug.c
 // Description: fichier associé �  la librairie debug.h
 // Programmeur: GAUCHER Alexis, étudiant isep
 // Création: 19 decembre 2007
 // Modifications: premiere version
 // Remarque: !! Ce fichier n'est compilé qu'en mode DEBUG (#define _DEBUG)
 ////////////////////////////////////////////////////////////////////////////////////

 #ifdef _DEBUG

 //////////////////////////// INCLUSIONS /////////////////////////////////////////

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdarg.h>
 #include <string.h>

 //////////////////////////// DEFINITIONS ///////////////////////////////////////

 #define FILE_LOG "log.txt"

 // liste chainée pour enregistrer les infos sur les malloc
 typedef struct liste{
 int line; // contiendra la ligne dans le programme qui a fait un malloc
 char file[100]; // contiendra le nom du fichier qui a fait le malloc
 void* adr; // pointeur vers la zone memoire allouée
 size_t n; // taille de la zone mémoire allouée

 struct liste *pSuivant;
 }liste;

 typedef liste* pliste;

 ////////////////////////////// GLOBALES ////////////////////////////////////////

 // tete de la liste chainée
 static pliste pHeadMalloc = NULL;
 // variable globale qui servira �  mémoriser brievement la ligne qui appelle malloc()
 static int gblVarErrorLine;
 // pour enregistrer le nom du fichier qui appelle le malloc()
 static char *gblVarErrorFile;

 ///////////////////////////// FONCTIONS ////////////////////////////////////////

 // pratique
 char* GetShortFileName(char* adr); // recupère le nom d'un fichier �  partir de son adresse ("usr/bin/toto" -> "toto")

 // gestion des malloc
 void ErrorLog(char *format, ...); // gestion des messages d'erreur
 void* _malloc(size_t n, int line, char* file); // allocation de la mémoire
 void _free(void* p, int line, char* file); // liberation de la mémoire
 void _closedebug(); // verifie les fuites memoires eventuelles

 // gestion des listes
 pliste NewAlloc(pliste *tete, int line, char* file, void* adr, size_t n); // nouvelle fiche
 pliste DelAlloc(pliste *tete); // supprimer la premiere fiche
 void DelTheAlloc(pliste *tete, void* p); // supprimer la fiche du pointeur p


 ////////////////////////////////////////////////////////////////////////////////////
 // 1. PETITES FONCTIONS PRATIQUES
 ////////////////////////////////////////////////////////////////////////////////////


 /* char* GetShortFileName(char* filename);
 ---------------------------------------------------------------------
 Description:
 extrait le nom du fichier de l'adresse passée en parametre.
 Parametre:
 char* adr: adresse du fichier (ex: /usr/bin/toto)
 retour:
 retourne un pointeur vers le nom du fichier
 Remarque :
 aucune.
 ------------------------------------------------------------------------ */
 char* GetShortFileName(char* adr)
 {
 char *p;
 char *pCur;

 p = pCur = adr;

 while (*pCur != '\0')
 {
 if (*pCur == '\\')
 p++;
 pCur++;
 }
 return p;

 } // end GetShortFileName

 ////////////////////////////////////////////////////////////////////////////////////
 // 2. GESTION DES MALLOC
 ////////////////////////////////////////////////////////////////////////////////////


 /* void ErrorLog(char *format, ...);
 ---------------------------------------------------------------------
 Description:
 sort un message d'erreur formaté �  l'ecran de la forme:
 "fichier.c(ligne): message d'erreur"
 Parametre:
 char *format: message d'erreur �  affiche/ecrire
 retour:
 aucune valeur retournée
 Exemple:
 ErrorLog("%d blocs memoires non liberes", 2);
 ------------------------------------------------------------------------ */
 void ErrorLog(char *format, ...)
 {
 static testFirstUse = 0;
 char buf[256], title[50];
 FILE* fileLog;

 // si c'est la première erreur, on cree un en-tête pour le fichier et on le vide (ou le cree)
 if (!testFirstUse) {
 fileLog = fopen(FILE_LOG, "w");
 fputs ("************* FICHIER DEBUG *************\n\n" , fileLog );
 fclose(fileLog);
 testFirstUse=1;
 }

 // titre de l'erreur de la forme: "main.c(105)"
 snprintf(title, sizeof(title), "%s(%d)", GetShortFileName( (gblVarErrorFile==NULL)? "fichier inconnu" : gblVarErrorFile ), gblVarErrorLine);

 va_list listArg;
 va_start(listArg,format);
 vsprintf(buf, format, listArg);
 va_end(listArg);

 if (fileLog = fopen(FILE_LOG, "a")) {
 fputs (title , fileLog );
 fputs (": " , fileLog );
 fputs (buf , fileLog );
 fputc (10, fileLog); // retour �  la ligne
 fclose (fileLog);
 }
 } // end ErrorLog


 /* void* _malloc(size_t n, int line, char* file);
 ---------------------------------------------------------------------
 Description:
 alloue de la mémoire et ajoute une fiche dans une liste chainée avec les valeurs suivantes :
 - l'adresse mémoire qui a été allouée
 - la quantité de mémoire allouée
 - le nom du fichier où se trouve la fonction qui a demandé cette action
 - la ligne �  lequelle se trouve cette fonction.
 Parametre:
 size_t n: taille du bloc memoire �  alouer
 int line: contient le numero de la ligne où se trouve la fonction appelante
 char* file: nom du fichier où se trouve la fonction appelante
 retour:
 retourne un pointeur vers le premier octet de l'espace alloué (comme malloc())
 Exemple:
 #include <stdlib.h>
 #include "debug.h"
 int main()
 {
 int* p;
 p = (int*) _malloc(sizeof(int), __LINE__, __FILE__);
 _free(p, __LINE__, __FILE__);
 ...
 _closedebug();
 return 0;
 }
 Remarque:
 Cette fonction doit être jumelée avec _free() et _closedebug().
 ------------------------------------------------------------------------ */
 void* _malloc(size_t n, int line, char* file)
 {
 void* p = NULL;
 p = (void*) malloc(n);
 if (p)
 NewAlloc(&pHeadMalloc, line, file, p, n);
 return p;
 } // end _malloc


 /* void _free(void* p);
 ---------------------------------------------------------------------
 Description:
 libère la mémoire allouée avec _malloc(), et supprime la fiche correspondante dans la
 liste chainée.
 Parametre:
 void* p: pointeur vers l'espace mémoire qu'il faut liberer.
 int line: contient le numero de la ligne où se trouve la fonction appelante
 char* file: nom du fichier où se trouve la fonction appelante
 retour:
 pas de valeur de retour
 Exemple:
 #include <stdlib.h>
 #include "debug.h"
 int main()
 {
 int* p;
 p = (int*) _malloc(sizeof(int), __LINE__, __FILE__);
 _free(p, __LINE__, __FILE__);
 ...
 _closedebug();
 return 0;
 }
 Remarque:
 Cette fonction doit être jumelée avec les fonctions _malloc() et _closedebug().
 ------------------------------------------------------------------------ */
 void _free(void* p, int line, char* file)
 {
 if (!p) {
 gblVarErrorFile = file;
 gblVarErrorLine = line;
 ErrorLog("liberation d'un pointeur deja nul");
 }
 else {
 DelTheAlloc(&pHeadMalloc, p);
 free(p);
 }
 } // end _free


 /* void _closedebug();
 ---------------------------------------------------------------------
 Description:
 Cette fonction doit-être appeler lorsque le programme se termine. Il regarde si
 certains blocs mémoires aloués avec _malloc() n'ont pas été désalloués
 (<=> il reste des fiches dans la liste chainée).
 Dans ce cas, la fonction ErrorLog() est appelée pour signaler le problème.
 Parametre:
 aucun parametre
 retour:
 pas de valeur de retour
 Exemple:
 #include <stdlib.h>
 #include "debug.h"
 int main()
 {
 int* p;
 p = (int*) _malloc(sizeof(int), __LINE__, __FILE__);
 _free(p, __LINE__, __FILE__);
 ...
 _closedebug();
 return 0;
 }
 Remarque:
 Cette fonction doit être jumelée avec _malloc() et _free().
 ------------------------------------------------------------------------ */
 void _closedebug()
 {
 while (pHeadMalloc) {
 gblVarErrorFile = pHeadMalloc->file;
 gblVarErrorLine = pHeadMalloc->line;
 ErrorLog("pointeur non libere (adresse: %p, nombre d'octets: %d)", pHeadMalloc->adr, pHeadMalloc->n);
 DelAlloc(&pHeadMalloc);
 }
 } // end _closedebug




 ////////////////////////////////////////////////////////////////////////////////////
 // 3. GESTION DE LA LISTE CHAINEE
 ////////////////////////////////////////////////////////////////////////////////////


 /* pliste NewAlloc(pliste *tete, int line, char* file, void* adr, size_t n)
 ---------------------------------------------------------------------
 Description:
 Cette fonction rajoute une fiche en dernière position �  la liste chainée.
 Si la liste ne possèdait aucune fiche, alors la première fiche est crée.
 Parametre:
 pliste *tete: pointeur vers le premier élément de la liste
 int line: |--
 char* file: | Données qui seront enregistrées dans la nouvelle fiche
 void* adr: | voir la fonction _malloc() plus haut.
 size_t n: |--
 retour:
 retourne un pointeur vers la nouvelle fiche
 ------------------------------------------------------------------------ */
 pliste NewAlloc(pliste *tete, int line, char* file, void* adr, size_t n)
 {
 pliste tmp = *tete;
 pliste nouv = (pliste) malloc(sizeof(liste));
 nouv->pSuivant=NULL;
 if (!nouv) return NULL;

 if (!*tete) // Si la liste était vide, alors nouv est la première fiche.
 *tete = nouv;
 else {
 while (tmp->pSuivant) // On fait pointer tmp vers la dernière fiche
 tmp = tmp->pSuivant;

 tmp->pSuivant = nouv; // la derniere fiche pointe vers nouv
 nouv->pSuivant = NULL; // nouv ne pointe vers aucune fiche (NULL)
 }
 nouv->line = line;
 strcpy(nouv->file, file);
 nouv->adr = adr;
 nouv->n = n;


 return nouv;
 } // end NewAlloc


 /* pliste DelAlloc(pliste *tete);
 ---------------------------------------------------------------------
 Description:
 Cette fonction supprime la première fiche de la liste chainée.
 Parametre:
 pliste *tete: pointeur vers le premier élément de la liste
 retour:
 retourne un pointeur vers la nouvelle fiche située en premiere position
 ------------------------------------------------------------------------ */
 pliste DelAlloc(pliste *tete)
 {
 liste tmp;

 if (!*tete)
 return NULL;
 tmp = **tete;
 free(*tete);
 *tete = tmp.pSuivant;
 return *tete;

 } // end DelAlloc


 /* void DelTheAlloc(pliste *tete, void* p);
 ---------------------------------------------------------------------
 Description:
 Cette fonction supprime la fiche de la liste chainée dont le champ adr vaut p.
 Parametre:
 pliste *tete: pointeur vers le premier élément de la liste
 void* p: adresse que l'on va comparer au champ adr. Ce sera la fiche qu'on supprimera
 retour:
 pas de valeur de retour
 ------------------------------------------------------------------------ */
 void DelTheAlloc(pliste *tete, void* p)
 {
 pliste prev = NULL;
 pliste current= *tete;
 if ((!tete)||(!p))
 return;
 else {
 while ((current->pSuivant)&&(current->adr != p)) {
 prev = current;
 current = current->pSuivant;
 }
 if (current->adr == p) {
 if (prev) {
 prev->pSuivant = current->pSuivant;
 free(current);
 }
 else
 DelAlloc(tete);
 }
 else
 return;
 }

 } // end DelTheAlloc

 #endif 

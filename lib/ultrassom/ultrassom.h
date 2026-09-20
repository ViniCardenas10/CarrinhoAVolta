#ifndef ULTRASSOM_H
#define ULTRASSOM_H


/* Torna a distância acessível para o main.c */
extern volatile float distancia_atual;

/* Protótipo da função que vai rodar na Thread */
void thread_ultrassom_func(void *arg1, void *arg2, void *arg3);

#endif /* ULTRASSOM_H */
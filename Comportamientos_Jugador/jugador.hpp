#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

struct estado {
  int fila;
  int columna;
  int orientacion;
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      ultimaAccion = actIDLE;
      hayPlan=false;
      seDondeEstoy=false;

      mapaAuxiliar.resize(mapaResultado.size()*2+1);
      vector<vector<unsigned char> >::iterator it;
      for(it=mapaAuxiliar.begin();it!=mapaAuxiliar.end();it++){
        it->resize(mapaResultado.size()*2+1,'?');
      }

      actualAuxiliar.fila=mapaResultado.size()+1;
      actualAuxiliar.columna=mapaResultado.size()+1;
      actualAuxiliar.orientacion=0;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      ultimaAccion = actIDLE;
      hayPlan=false;
      seDondeEstoy=false;

      mapaAuxiliar.resize(mapaResultado.size()*2);
      vector<vector<unsigned char> >::iterator it;
      for(it=mapaAuxiliar.begin();it!=mapaAuxiliar.end();it++){
        it->resize(mapaResultado.size()*2,'?');
      }
      actualAuxiliar.fila=mapaResultado.size()+1;
      actualAuxiliar.columna=mapaResultado.size()+1;
      actualAuxiliar.orientacion=0;

    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    int fil, col, brujula;
    estado actual, destino;
    list<Action> plan;

    Action ultimaAccion;
    bool hayPlan;
    vector<vector<unsigned char> > mapaAuxiliar;
    estado actualAuxiliar;
    bool seDondeEstoy;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Coste_uniforme(const estado &origen, const estado &destino, list<Action> &plan);
    void actualizarMapa(int fil, int col, int brujula,Sensores &sensores,vector<vector<unsigned char> > &map);
    void actualizarMapa(estado &act, estado &actAux, vector<vector<unsigned char> > &mapAux, vector<vector<unsigned char> > &mapR);

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);

};

#endif

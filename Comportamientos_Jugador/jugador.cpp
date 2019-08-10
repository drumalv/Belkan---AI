#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>
#include <vector>
#include <iterator>

// Este es el método principal que debe contener los 4 Comportamientos_Jugador
// que se piden en la práctica. Tiene como entrada la información de los
// sensores y devuelve la acción a realizar.
Action ComportamientoJugador::think(Sensores sensores) {


	if(sensores.nivel!=4){
		Action sigAccion = actIDLE;
		// Estoy en el nivel 1

		if (sensores.mensajeF != -1){
			fil = sensores.mensajeF;
			col = sensores.mensajeC;
			ultimaAccion= actIDLE;
		}

		//Analizar el efecto de la ultima accion
		switch(ultimaAccion){
			case actTURN_R: brujula = (brujula+1)%4; break;
			case actTURN_L: brujula = (brujula+3)%4; break;
			case actFORWARD:
				switch (brujula) {
					case 0: fil--; break;
					case 1: col++; break;
					case 2: fil++; break;
					case 3: col--; break;
				}
				break;
		}

		//Mirar si ha cambiado el destino
		if(sensores.destinoF != destino.fila || sensores.destinoC != destino.columna){
			destino.fila=sensores.destinoF;
			destino.columna=sensores.destinoC;
			hayPlan = false;
		}

		//Calcular un camino hasta el destino
		if(!hayPlan){
			actual.fila=fil;
			actual.columna=col;
			actual.orientacion=brujula;
			hayPlan=pathFinding(sensores.nivel,actual,destino,plan);
		}

		if(hayPlan && plan.size()>0){ //hay plan y hay que seguirlo
			sigAccion=plan.front();
			plan.erase(plan.begin());
		}else{							//No hay plan y se activa Comportamiento reactivo
			if(sensores.terreno[2]=='P' || sensores.terreno[2]=='M' ||
				 sensores.terreno[2]=='D' || sensores.superficie[2]=='a'){
					 	sigAccion=actTURN_R;
			}else{
				sigAccion=actFORWARD;
			}
		}

		//Recordar la última accion
		ultimaAccion=sigAccion;
		return sigAccion;

	}else{

		//reto 2

		Action sigAccion = actIDLE;

		if (sensores.mensajeF != -1){
			fil = sensores.mensajeF;
			col = sensores.mensajeC;
			ultimaAccion=actIDLE;
		}

		if(!hayPlan && !seDondeEstoy){
			if(sensores.terreno[0]=='K'){
				actual.fila=fil;
				actual.columna=col;
				actual.orientacion=brujula;
				seDondeEstoy=true;

				actualizarMapa(actual,actualAuxiliar,mapaAuxiliar,mapaResultado);
				//actualizarMapa(fil,col,brujula,sensores,mapaResultado);
				hayPlan=pathFinding(sensores.nivel,actual,destino,plan);

			}else if(sensores.terreno[2]=='P' || sensores.terreno[2]=='M' ||
				 sensores.terreno[2]=='D' || sensores.superficie[2]=='a'){
					 	sigAccion=actTURN_R;
			}else{
				sigAccion=actFORWARD;
			}


			//Analizar el efecto de la ultima accion en actualAuxiliar
			switch(ultimaAccion){
				case actTURN_R: actualAuxiliar.orientacion = (actualAuxiliar.orientacion+1)%4; break;
				case actTURN_L: actualAuxiliar.orientacion = (actualAuxiliar.orientacion+3)%4; break;
				case actFORWARD:
					switch (actualAuxiliar.orientacion) {
						case 0: actualAuxiliar.fila--; break;
						case 1: actualAuxiliar.columna++; break;
						case 2: actualAuxiliar.fila++; break;
						case 3: actualAuxiliar.columna--; break;
					}
					break;
			}

			actualizarMapa(actualAuxiliar.fila,actualAuxiliar.columna,actualAuxiliar.orientacion,sensores,mapaAuxiliar);

		}

		//Analizar el efecto de la ultima accion
		switch(ultimaAccion){
			case actTURN_R: brujula = (brujula+1)%4; break;
			case actTURN_L: brujula = (brujula+3)%4; break;
			case actFORWARD:
				switch (brujula) {
					case 0: fil--; break;
					case 1: col++; break;
					case 2: fil++; break;
					case 3: col--; break;
				}
				break;
		}
		//Mirar si ha cambiado el destino
		if(sensores.destinoF != destino.fila || sensores.destinoC != destino.columna){
			destino.fila=sensores.destinoF;
			destino.columna=sensores.destinoC;
			hayPlan = false;
		}

		if(!hayPlan && seDondeEstoy){
			actual.fila=fil;
			actual.columna=col;
			actual.orientacion=brujula;
			hayPlan=pathFinding(sensores.nivel,actual,destino,plan);
		}

		if(hayPlan && plan.size()>0 && seDondeEstoy){ //hay plan y hay que seguirlo
			if(sensores.terreno[2]=='P' || sensores.terreno[2]=='M' || sensores.terreno[2]=='D'){

				if(plan.front()==actFORWARD){
						if(ultimaAccion==actTURN_L){
							sigAccion=actTURN_L;
						}else
							sigAccion=actTURN_R;
						hayPlan=false;
				}else{
						sigAccion=plan.front();
						plan.erase(plan.begin());
						actualizarMapa(fil,col,brujula,sensores,mapaResultado);
				}
			}else if (sensores.superficie[2]=='a'){
					sigAccion=actIDLE;
			}else{
				sigAccion=plan.front();
				plan.erase(plan.begin());
				actualizarMapa(fil,col,brujula,sensores,mapaResultado);

			}
		}

		//Recordar la última accion
		ultimaAccion=sigAccion;
		return sigAccion;
	}


}
void ComportamientoJugador::actualizarMapa(estado &act, estado &actAux, vector<vector<unsigned char> > &mapAux, vector<vector<unsigned char> > &mapR){

	int Fmin=99999999,Fmax=0,Cmin=9999999,Cmax=0;								//Rectangulo escrito en mapa auxiliar

	for(int i=0;i<mapAux.size();i++){
		for(int j=0;j<mapAux.size();j++){
			if(mapAux[i][j]!='?'){
				if(i>Fmax)Fmax=i;
				if(i<Fmin)Fmin=i;
				if(j>Cmax)Cmax=j;
				if(j<Cmin)Cmin=j;
			}
		}
	}

	int DFmin, DCmin, DFmax, DCmax;				//Distancias con respecto actual posicion auxiliar
	DFmin=actAux.fila-Fmin;			DCmin=actAux.columna-Cmin;
	DFmax=Fmax-actAux.fila;			DCmax=Cmax-actAux.columna;

	int h=act.fila-DFmin-1;
	for(int i=Fmin; i<Fmax; i++){
		int k=act.columna-DCmin;
		for(int j=Cmin; j<=Cmax; j++){
			mapR[h][k]=mapAux[i][j];
			k++;
		}
		h++;
	}

}

void ComportamientoJugador::actualizarMapa(int fil, int col, int brujula,Sensores &sensores, vector<vector<unsigned char> > &map){

		switch (brujula) {
			case 0://norte
				map[fil][col]=sensores.terreno[0];map[fil-1][col-1]=sensores.terreno[1];
				map[fil-1][col]=sensores.terreno[2];map[fil-1][col+1]=sensores.terreno[3];
				map[fil-2][col-2]=sensores.terreno[4];map[fil-2][col-1]=sensores.terreno[5];
				map[fil-2][col]=sensores.terreno[6];map[fil-2][col+1]=sensores.terreno[7];
				map[fil-2][col+2]=sensores.terreno[8];map[fil-3][col-3]=sensores.terreno[9];
				map[fil-3][col-2]=sensores.terreno[10];map[fil-3][col-1]=sensores.terreno[11];
				map[fil-3][col]=sensores.terreno[12];map[fil-3][col+1]=sensores.terreno[13];
				map[fil-3][col+2]=sensores.terreno[14];map[fil-3][col+3]=sensores.terreno[15];
				break;
			case 1://este
				map[fil][col]=sensores.terreno[0];map[fil-1][col+1]=sensores.terreno[1];
				map[fil][col+1]=sensores.terreno[2];map[fil+1][col+1]=sensores.terreno[3];
				map[fil-2][col+2]=sensores.terreno[4];map[fil-1][col+2]=sensores.terreno[5];
				map[fil][col+2]=sensores.terreno[6];map[fil+1][col+2]=sensores.terreno[7];
				map[fil+2][col+2]=sensores.terreno[8];map[fil-3][col+3]=sensores.terreno[9];
				map[fil-2][col+3]=sensores.terreno[10];map[fil-1][col+3]=sensores.terreno[11];
				map[fil][col+3]=sensores.terreno[12];map[fil+1][col+3]=sensores.terreno[13];
				map[fil+2][col+3]=sensores.terreno[14];map[fil+3][col+3]=sensores.terreno[15];
				break;
			case 2://sur
				map[fil][col]=sensores.terreno[0];map[fil+1][col+1]=sensores.terreno[1];
				map[fil+1][col]=sensores.terreno[2];map[fil+1][col-1]=sensores.terreno[3];
				map[fil+2][col+2]=sensores.terreno[4];map[fil+2][col+1]=sensores.terreno[5];
				map[fil+2][col]=sensores.terreno[6];map[fil+2][col-1]=sensores.terreno[7];
				map[fil+2][col-2]=sensores.terreno[8];map[fil+3][col+3]=sensores.terreno[9];
				map[fil+3][col+2]=sensores.terreno[10];map[fil+3][col+1]=sensores.terreno[11];
				map[fil+3][col]=sensores.terreno[12];map[fil+3][col-1]=sensores.terreno[13];
				map[fil+3][col-2]=sensores.terreno[14];map[fil+3][col-3]=sensores.terreno[15];
				break;
			case 3://oeste
				map[fil][col]=sensores.terreno[0];map[fil+1][col-1]=sensores.terreno[1];
				map[fil][col-1]=sensores.terreno[2];map[fil-1][col-1]=sensores.terreno[3];
				map[fil+2][col-2]=sensores.terreno[4];map[fil+1][col-2]=sensores.terreno[5];
				map[fil][col-2]=sensores.terreno[6];map[fil-1][col-2]=sensores.terreno[7];
				map[fil-2][col-2]=sensores.terreno[8];map[fil+3][col-3]=sensores.terreno[9];
				map[fil+2][col-3]=sensores.terreno[10];map[fil+1][col-3]=sensores.terreno[11];
				map[fil][col-3]=sensores.terreno[12];map[fil-1][col-3]=sensores.terreno[13];
				map[fil-2][col-3]=sensores.terreno[14];map[fil-3][col-3]=sensores.terreno[15];
				break;
		}

}



// Llama al algoritmo de busqueda que se usará en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (int level, const estado &origen, const estado &destino, list<Action> &plan){
	switch (level){
		case 1: cout << "Busqueda en profundad\n";
			      return pathFinding_Profundidad(origen,destino,plan);
						break;
		case 2: cout << "Busqueda en Anchura\n";
			      return pathFinding_Anchura(origen,destino,plan);
						break;
		case 3: cout << "Busqueda Costo Uniforme\n";
						return pathFinding_Coste_uniforme(origen,destino,plan);
						break;
		case 4: cout << "Busqueda para el reto\n";
						return pathFinding_Coste_uniforme(origen,destino,plan);
						break;
	}
	cout << "Comportamiento sin implementar\n";
	return false;
}


//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el código en carácter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla){
	if (casilla=='P' or casilla=='M' or casilla =='D')
		return true;
	else
	  return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
	int fil=st.fila, col=st.columna;

  // calculo cual es la casilla de delante del agente
	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil<0 or fil>=mapaResultado.size()) return true;
	if (col<0 or col>=mapaResultado[0].size()) return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col])){
		// No hay obstaculo, actualizo el parámetro st poniendo la casilla de delante.
    st.fila = fil;
		st.columna = col;
		return false;
	}
	else{
	  return true;
	}
}




struct nodo{
	estado st;
	list<Action> secuencia;
	int peso;

	nodo(){
		peso=0;
	}

};

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion))
			return true;
		else
			return false;
	}
};

//Implementación de la busqueda en anchura.
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan){

	cout<< "Calculando plan anchura ..." << endl;
	plan.clear();
	queue<nodo> colaNodos;								// nodos sin explorar
	set<estado,ComparaEstados> generados;

	nodo solucion;
	solucion.peso=99999999;
	nodo current;
	current.st = origen;
	current.secuencia.empty();

	colaNodos.push(current);

	while(!colaNodos.empty()){

		colaNodos.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		hijoTurnR.peso++;
		if (hijoTurnR.peso < solucion.peso && generados.find(hijoTurnR.st)==generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			colaNodos.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		hijoTurnL.peso++;
		if (hijoTurnL.peso < solucion.peso && generados.find(hijoTurnL.st)==generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			colaNodos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if(hijoForward.st.fila!=current.st.fila || hijoForward.st.columna!=current.st.columna){
				switch (mapaResultado[hijoForward.st.fila][hijoForward.st.columna]) {
					case 'T': hijoForward.peso += 2; break;
					case 'B': hijoForward.peso += 5; break;
					case 'A': hijoForward.peso += 10; break;
					default : hijoForward.peso++; break;
				}
			}
			if (hijoForward.peso < solucion.peso && generados.find(hijoForward.st)==generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				colaNodos.push(hijoForward);
			}
		}

		if (!colaNodos.empty()){
			current = colaNodos.front();
		}

		if ( current.st.fila == destino.fila && current.st.columna == destino.columna && current.peso < solucion.peso){
				solucion.st=current.st;
				cout<<endl<<"actualizo solucion"<<endl;
				solucion.peso=current.peso;
				solucion.secuencia=current.secuencia;
		}

	}

	cout << "Terminada la busqueda\n";

	if (solucion.st.fila == destino.fila && solucion.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = solucion.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


}

struct ComparaPesosNodos{
	bool operator()(const nodo &a, const nodo &n) const{
		if (a.peso > n.peso)
			return true;
		else
			return false;
	}
};

bool ComportamientoJugador::pathFinding_Coste_uniforme(const estado &origen, const estado &destino, list<Action> &plan){

	cout<< "Calculando plan coste uniforme ..." << endl;
	plan.clear();
	priority_queue<nodo,vector<nodo>,ComparaPesosNodos> colaNodos;								// nodos sin explorar
	set<estado,ComparaEstados> generados;

	nodo current;
	current.st = origen;
	current.secuencia.empty();

	colaNodos.push(current);

	while(!colaNodos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		colaNodos.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		hijoTurnR.peso++;
		if (generados.find(hijoTurnR.st)==generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			colaNodos.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		hijoTurnL.peso++;
		if (generados.find(hijoTurnL.st)==generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			colaNodos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){

			switch (mapaResultado[hijoForward.st.fila][hijoForward.st.columna]) {
				case 'T': hijoForward.peso += 2; break;
				case 'B': hijoForward.peso += 5; break;
				case 'A': hijoForward.peso += 10; break;
				default : hijoForward.peso++; break;
			}

			if (generados.find(hijoForward.st)==generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				colaNodos.push(hijoForward);
			}
		}


		if (!colaNodos.empty()){
			current = colaNodos.top();
		}

	}

	cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila && current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


}



// Implementación de la búsqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; // Lista de Cerrados
	stack<nodo> pila;											// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	pila.push(current);

  while (!pila.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		pila.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			pila.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			pila.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				pila.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la pila
		if (!pila.empty()){
			current = pila.top();
		}


	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}







// Sacar por la términal la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			cout << "A ";
		}
		else if (*it == actTURN_R){
			cout << "D ";
		}
		else if (*it == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}



void AnularMatriz(vector<vector<unsigned char> > &m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}


// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			switch (cst.orientacion) {
				case 0: cst.fila--; break;
				case 1: cst.columna++; break;
				case 2: cst.fila++; break;
				case 3: cst.columna--; break;
			}
			mapaConPlan[cst.fila][cst.columna]=1;
		}
		else if (*it == actTURN_R){
			cst.orientacion = (cst.orientacion+1)%4;
		}
		else {
			cst.orientacion = (cst.orientacion+3)%4;
		}
		it++;
	}
}



int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

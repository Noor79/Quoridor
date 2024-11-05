using Microsoft.Xna.Framework;
using System.Collections.Generic;


//Brädet är numrerat från nedre vänstra hörnet (0,0) till övre högra hörnet (8,8)
//Spelaren går alltid uppåt dvs (-,8) är målraden.

public enum Typ { Flytta, Horisontell, Vertikal }
public struct Drag {
    public Typ typ;
    public Point point;
}
public class SpelBräde {
    public const int N = 9;
    public bool avanceradeRegler;       //Normalt false
    public List<Spelare> spelare;       //Spelaren är alltid först i listan.

    //Horisontella väggar, om (x,y) är satt
    // så kan spelaren inte gå från (x,y) till (x,y+1)
    public bool[,] horisontellaVäggar = new bool[N, N - 1];  //9*8

    //Vertikala väggar, om (x,y) är satt
    // så kan spelaren inte gå från (x,y) till (x+1,y)
    public bool[,] vertikalaVäggar = new bool[N - 1, N]; //8*9

    #region Överkurs
    //Om man vill hindra väggarna från att korsas.
    //Horisontella långa väggar, om (x,y) är satt 
    // så kan spelaren inte gå från(x, y) till(x, y+1)
    //          och inte från (x+1, y) till (x+1, y+1)
    public bool[,] horisontellaLångaVäggar = new bool[N - 1, N - 1]; //8*8

    //Vertikala långa väggar, om (x,y) är satt
    // så kan spelaren inte gå från (x,y) till (x+1,y)
    //          och inte från (x, y+1) till (x+1, y+1)
    public bool[,] vertikalaLångaVäggar = new bool[N - 1, N - 1]; //8*8
    #endregion Överkurs
    public SpelBräde() {    }
}
public enum Färg { Röd, Blå };  //Röd börjar alltid
public class Spelare {
    public Point position;  //Platsen spelaren eller väggen ska placeras
    public Färg färg;
    public int antalVäggar; //Antal väggar kvar att placera ut
}

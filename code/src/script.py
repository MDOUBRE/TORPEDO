

from termios import OFDEL
print("To skip non existing sommet enter : end")
mat = []

taille = 20
for x in range(0,taille):
    tmp = [-1,-1,-1,-1]
    mat.append(tmp)
for x in range(0,taille):
    if(mat[x][0] == -1):
        s = input("N du sommet "+str(x)+"\n")
        if (s =="end"):
            break
        s = int(s)
        mat[x][0] = s
        if (mat[s][2]== -1 and s != -1):
            mat[s][2]=x

    if(mat[x][1] == -1):
        s = input("E du sommet "+str(x)+"\n")
        if (s =="end"):
            break
        s = int(s)
        mat[x][1] = s
        if (mat[s][3]== -1 and s != -1):
            mat[s][3]=x

    if(mat[x][2] == -1):
        s = input("S du sommet "+str(x)+"\n")
        if (s =="end"):
            break
        s = int(s)
        mat[x][2] = s
        if (mat[s][0]== -1 and s != -1):
            mat[s][0]=x

    if(mat[x][3] == -1):
        s = input("O du sommet "+str(x)+"\n")
        if (s =="end"):
            break
        s = int(s)
        mat[x][3] = s
        if (mat[s][1]== -1 and s != -1):
            mat[s][1]=x

f = open("mat.txt", "w")
f.write("")
f.close()
f = open("mat.txt", "a")
for x in range(0,taille):
    f.write("grille["+str(x)+"].nord = "+str(mat[x][0])+";\n")
    f.write("grille["+str(x)+"].est = "+str(mat[x][1])+";\n")
    f.write("grille["+str(x)+"].sud = "+str(mat[x][2])+";\n")
    f.write("grille["+str(x)+"].ouest = "+str(mat[x][3])+";\n")
f.close()
print("File mat.txt created")

import pygame, heapq, time

"""Obstacles={
    "Obstacle1":[7,3],
    "Obstacle2":[5,8],
    "Obstacle3":[1,4],
    "Obstacle4":[2,6],
    "Obstacle5":[5,1],

Obstacles={
    "Obstacle1":[7,4],
    "Obstacle2":[6,3],
    "Obstacle3":[5,1],
    "Obstacle4":[4,7],
    "Obstacle5":[8,2],
}
Obstacles={
    "Obstacle1":[1,3],
    "Obstacle2":[6,6],
    "Obstacle3":[4,1],
    "Obstacle4":[8,2],
    "Obstacle5":[8,4],
}
Obstacles={
    "Obstacle1":[5,3],
    "Obstacle2":[6,2],
    "Obstacle3":[7,2],
    "Obstacle4":[8,2],
    "Obstacle5":[6,4],
    "Obstacle6":[2,6],
    "Obstacle7":[2,8],
    "Obstacle8":[3,7],
    "Obstacle9":[1,4],
    "Obstacle10":[5,6],
}"""
Obstacles={
    "Obstacle1":[5,3],
    "Obstacle2":[6,2],
    "Obstacle3":[7,2],
    "Obstacle4":[8,2],
    "Obstacle5":[6,4],
    "Obstacle6":[2,6],
    "Obstacle7":[2,8],
    "Obstacle8":[3,7],
    "Obstacle9":[1,4],
    "Obstacle10":[5,6],
    "Obstacle11":[3,2],
    "Obstacle12":[3,4],
    "Obstacle13":[4,1],
    "Obstacle14":[6,5],
    "Obstacle15":[7,7],
}

Blocks={
"Block1":[4,2],
"Block2":[2,7],
}

image=pygame.image.load("D:\\Personal_Misc\\Region 5 Robotics\\2018-2019\\Mock Bot.png")
display_w=800
display_h=800
Display=pygame.display.set_mode((display_w,display_h))
faux_pos=[1,1]

Block_Pickup=0
Pos=[1,1]
Mothership=[6,3]

Block="no"

Block_Pickup=0
class Cell():
    def __init__(self,x,y,reachable):
        self.reachable=reachable
        self.x=x
        self.y=y
        self.parent=None
        self.g=0
        self.h=0
        self.f=0

    def __lt__(self,other):
        return self.f<other.f

class AStar():
    def __init__(self,start,end):
        self.start=start
        self.end=end
        self.opened=[]
        heapq.heapify(self.opened)
        #list of visited cells
        self.closed=set()
        #grid cells
        self.cells=[]
        self.width=10
        self.height=10
        self.grid_height=8
        self.grid_width=8
        self.tolerance=1
        self.Grid(start,end)
        self.Solver(start,end)

    def Grid(self,start,end):
        print('hello')
        obstacles=[]
        for obstacle in Obstacles.items():
            obstacles.append(obstacle[1])
        for x in range(1,8):
            obstacles.append([0,x])
            obstacles.append([9,x])
            obstacles.append([x,0])
            obstacles.append([x,9])
        #print(obstacles)    
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if [x,y] in obstacles:
                    reachable=False
                else:
                    reachable=True
                    #print(reachable)
                self.cells.append(Cell(x,y,reachable))
            print(x)
        self.start=self.get_cell(*start)
        self.end=self.get_cell(*end)
                
    def adjacent_cells(self,cell):
        cells=[]
        """neighbors=[[0,1],[1,0],[0,-1],[-1,0]]
        for n in neighbors:
            cells.append(self.get_cell(cell.x+n[0],cell.y+n[1]))"""
        if cell.x<self.grid_width-1:
            cells.append(self.get_cell(cell.x+1,cell.y))
        if cell.y>0:
            cells.append(self.get_cell(cell.x,cell.y-1))
        if cell.x>0:
            cells.append(self.get_cell(cell.x-1,cell.y))
        if cell.y<self.grid_height-1:
            cells.append(self.get_cell(cell.x,cell.y+1))
        return(cells)

    def path(self,start):
        cell=self.end
        path=[(cell.x,cell.y)]
        while cell.parent is not self.start:
            cell=cell.parent
            path.append((cell.x,cell.y))
        path.append((self.start.x,self.start.y))
        path.reverse()
        print(path)
        x=start[0]
        y=start[1]
        for stuff in path:
            if x==stuff[0] and stuff[1]>y:
                path_angle=180
            elif x==stuff[0] and stuff[1]<y:
                path_angle=0
            elif stuff[1]==y and stuff[0]>x:
                path_angle=270
            else:
                path_angle=90
            #-------------------------
            turn=path_angle
            #Turns ccw
            #print('----')
            #print(turn)
            image1=pygame.transform.rotate(image,turn)
            Display.blit(image1,(stuff[0]*100-100,stuff[1]*100-100))
            time.sleep(0.3)
            x=stuff[0]
            y=stuff[1]
            pygame.display.update()
        return(path)
    
    def get_h(self,cell):
        return(10*(abs(cell.x-self.end.x)+abs(cell.y-self.end.y)))
    
    def get_cell(self,x,y):
        return(self.cells[x*(self.grid_height)+y])
    
    def update_cell(self,adj,cell):
        adj.g=cell.g+10
        adj.h=self.get_h(adj)
        adj.parent=cell
        adj.f=adj.h+adj.g

    def Solver(self,start,end):
        heapq.heappush(self.opened, (self.start.f, self.start))
        while len(self.opened):
            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, return found path
            if cell is self.end:
                return self.path(start)
            # get adjacent cells for cell
            adj_cells = self.adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        if adj_cell.g > cell.g + 1:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))
            
            

def Display1():
        Display.fill((255,255,255))
        pygame.draw.rect(Display, (0,0,0),(0,0,display_w,display_h),1)
        for y in range(0,display_h,100):
            y+=100
            pygame.draw.line(Display,(0,0,0), (0,y),(display_w,y),1)
        for x in range(0,display_w,100):
            x+=100
            pygame.draw.line(Display,(0,0,0), (x,0),(x,display_h),1)
            #pygame.display.update()
        for obstacle in Obstacles.items():
            x1=obstacle[1][0]
            y1=obstacle[1][1]
            pygame.draw.rect(Display, (0,255,0), ((x1*100)-100,(y1*100)-100,100,100))
            #pygame.display.update()
        for blocks in Blocks.items():
            x2=blocks[1][0]
            y2=blocks[1][1]
            pygame.draw.rect(Display,(0,0,255),((x2*100)-100,(y2*100)-100,100,100))
        pygame.draw.rect(Display,(255,0,0),(Mothership[0]*100-100,Mothership[1]*100-100,100,100))
        pygame.display.update()
        
def Small_Game():
        x=0
        y=0
        display_w=800
        display_h=800
        Display=pygame.display.set_mode((display_w,display_h))
        Block_Pickup=0
        Pos=[1,1]
        Mothership=[6,3]
        for block in Blocks.items():
            Block="no"
            while Block=="no":
                Display1()
                start=(Pos[0],Pos[1])
                end=(block[1][0],block[1][1])
                path=AStar(start,end)
                print(path)
                print('hi')
                Block="yes"
            while Block=="yes":
                Display1()
                print('bye')
                start=(block[1][0],block[1][1])
                end=(Mothership[0],Mothership[1])
                AStar(start,end)
                print(path)
                Pos=Mothership
                Block="done"
            

Small_Game()

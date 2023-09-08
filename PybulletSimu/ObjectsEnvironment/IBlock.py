class IBlock():  # blocs les uns au dessus des autres, fait pour que l'ia apprenne à sauter dessus pour passer des obstacles par exemple
    def __init__(self, material, height=1):
        self.material = material
        self.height = height

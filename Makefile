docker-verison = 1.1

init:
	pip3 install -r requirements.txt

dockerbuild:
	docker build -t quixsam:$(docker-verison) .

dockerrun:
	docker run -i --net=host -v C:\Users\cttew\Desktop\quixsam:/code quixsam:$(docker-verison)

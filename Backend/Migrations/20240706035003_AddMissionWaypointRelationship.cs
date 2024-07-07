using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class AddMissionWaypointRelationship : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Waypoint_Missions_MissionId",
                table: "Waypoint");

            migrationBuilder.AlterColumn<int>(
                name: "MissionId",
                table: "Waypoint",
                type: "INTEGER",
                nullable: false,
                defaultValue: 0,
                oldClrType: typeof(int),
                oldType: "INTEGER",
                oldNullable: true);

            migrationBuilder.AddForeignKey(
                name: "FK_Waypoint_Missions_MissionId",
                table: "Waypoint",
                column: "MissionId",
                principalTable: "Missions",
                principalColumn: "Id",
                onDelete: ReferentialAction.Cascade);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Waypoint_Missions_MissionId",
                table: "Waypoint");

            migrationBuilder.AlterColumn<int>(
                name: "MissionId",
                table: "Waypoint",
                type: "INTEGER",
                nullable: true,
                oldClrType: typeof(int),
                oldType: "INTEGER");

            migrationBuilder.AddForeignKey(
                name: "FK_Waypoint_Missions_MissionId",
                table: "Waypoint",
                column: "MissionId",
                principalTable: "Missions",
                principalColumn: "Id");
        }
    }
}
